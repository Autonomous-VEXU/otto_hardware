#include "otto_hardware/omni_wheel_hardware.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>  
#include <iostream>
#include <vector>
#include <cstring>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>

#include "seriallib.h"
#include "otto_hardware/v5_protocol.hpp"  // the message format, shared with the V5 Brain program

serialib serial;

namespace otto_hardware {

    static std::string serial_port = "/dev/ttyTHS1";
    static int baud = 115200;

    static std::vector<uint8_t> rx_buf;

    // Index of each wheel in pos_ / vel_ / cmd_. Same order as joint_names_ in the header
    // (counterclockwise, the order the omni wheel controller uses). The message fields are
    // filled BY NAME from these, so the order here and in the message can't get mixed up.
    constexpr size_t FL = 0;  // front_left_wheel_joint
    constexpr size_t BL = 1;  // back_left_wheel_joint
    constexpr size_t BR = 2;  // back_right_wheel_joint
    constexpr size_t FR = 3;  // front_right_wheel_joint


    hardware_interface::CallbackReturn OmniWheelHardware::on_init(hardware_interface::HardwareComponentInterfaceParams & params){
        /*  Things that need to occur:
            - URDF ros2_control tag parameters are put into variables 
            - verify that the joints have the correct number and type of command/state interfaces
        */

        if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS){
            return hardware_interface::CallbackReturn::ERROR;
        }

        // initialize arrays to zero
        for (size_t i = 0; i < 4; ++i) {
            pos_[i] = 0.0;
            vel_[i] = 0.0;
            cmd_[i] = 0.0;
        }

        // initialize arrays to zero
        for (size_t i = 0; i < 3; ++i) {
            intake_pos_[i] = 0.0;
            intake_vel_[i] = 0.0;
            intake_cmd_[i] = 0.0;
        }

        auto it = info_.hardware_parameters.find("serial_port"); 

        if (it != info_.hardware_parameters.end()){
            serial_port = it->second;  // get the value from URDF (it->first is the "key" or param name)
        } else{
            RCLCPP_WARN(rclcpp::get_logger("OmniWheelHardware"), "serial_port not found, using: %s", serial_port.c_str());
        }

        it = info_.hardware_parameters.find("baud_rate");
        if (it != info_.hardware_parameters.end()) {
            baud = std::stoi(it->second); 
        } else{
            RCLCPP_WARN(rclcpp::get_logger("OmniWheelHardware"), "baud_rate not found, using: %d", baud);
        }

        // each joint should have 2 state interfaces (position + velocity) and one command interface (velocity)
        for (const hardware_interface::ComponentInfo & joint : info_.joints){ 

            if (joint.command_interfaces.size() != 1){ // verify number of command interfaces
                RCLCPP_FATAL(rclcpp::get_logger("OmniWheelHardware"), "Joint '%s' has %zu command interfaces. Expected 1.",joint.name.c_str(), joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY){ // verify type of command interfaces
                RCLCPP_FATAL(rclcpp::get_logger("OmniWheelHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",joint.name.c_str(), joint.command_interfaces[0].name.c_str(),hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2){ // verify number of state interfaces
                RCLCPP_FATAL(rclcpp::get_logger("OmniWheelHardware"), "Joint '%s' has %zu state interfaces. Expected 2.",joint.name.c_str(), joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION || joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY){// verify type of state interface
                RCLCPP_FATAL(rclcpp::get_logger("OmniWheelHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",joint.name.c_str(), joint.state_interfaces[0].name.c_str(),hardware_interface::HW_IF_POSITION);
                RCLCPP_FATAL(rclcpp::get_logger("OmniWheelHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",joint.name.c_str(), joint.state_interfaces[1].name.c_str(),hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Initialization complete!");

        return hardware_interface::CallbackReturn::SUCCESS; // success msg
    }

    std::vector<hardware_interface::StateInterface> OmniWheelHardware::export_state_interfaces(){
        std::vector<hardware_interface::StateInterface> interfaces;
        for (size_t i = 0; i < 4; ++i){
            interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_[i]);
            interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_[i]);
        }
        for (size_t i = 0; i < 3; i++){
            interfaces.emplace_back(intake_joint_names_[i], hardware_interface::HW_IF_POSITION, &intake_pos_[i]);
            interfaces.emplace_back(intake_joint_names_[i], hardware_interface::HW_IF_VELOCITY, &intake_vel_[i]);
        }
        return interfaces;
    }

    std::vector<hardware_interface::CommandInterface> OmniWheelHardware::export_command_interfaces(){
        std::vector<hardware_interface::CommandInterface> interfaces;
        for (size_t i = 0; i < 4; ++i){
            interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &cmd_[i]);
        }

        for(size_t i = 0; i<3; i++){
            interfaces.emplace_back(intake_joint_names_[i], hardware_interface::HW_IF_VELOCITY, &intake_cmd_[i]);
        }
        return interfaces;
    }

    hardware_interface::return_type OmniWheelHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
        RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "read() called");
        
        if (!serial.isDeviceOpen()){ 
            RCLCPP_ERROR(rclcpp::get_logger("OmniWheelHardware"), "Device not open, cannot read data");
            return hardware_interface::return_type::ERROR;
        }
        
        // 1. Move every byte that has arrived from the Brain into rx_buf.
        int available = serial.available();
        if (available > 0) {
            uint8_t temp_buf[512];
            int bytes_read = serial.readBytes(temp_buf, std::min(available, (int)sizeof(temp_buf)));
            if (bytes_read > 0) {
                rx_buf.insert(rx_buf.end(), temp_buf, temp_buf + bytes_read);
            }
        }

        // 2. Use EVERY complete message in rx_buf, oldest first, so the newest values win and
        //    nothing piles up. Bytes that aren't part of a message are skipped.
        const size_t used = v5_protocol::parse_stream(rx_buf.data(), rx_buf.size(),
            [this](uint64_t magic, const uint8_t* bytes) {
                if (magic != v5_protocol::POSITIONS_MAGIC && magic != v5_protocol::SPEEDS_MAGIC) {
                    return;  // only Positions and Speeds travel Brain -> Jetson
                }
                const v5_protocol::Packet p = v5_protocol::read_packet(bytes);
                const bool is_position = (magic == v5_protocol::POSITIONS_MAGIC);
                std::array<double, 4>& wheel = is_position ? pos_ : vel_;
                std::array<double, 3>& intake = is_position ? intake_pos_ : intake_vel_;

                wheel[FL] = p.front_left;
                wheel[BL] = p.back_left;
                wheel[BR] = p.back_right;
                wheel[FR] = p.front_right;
                intake[0] = p.intake1;
                intake[1] = p.intake2;
                intake[2] = p.intake3;
            });
        rx_buf.erase(rx_buf.begin(), rx_buf.begin() + used);

        RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"),
            "pos: fl=%.2f bl=%.2f br=%.2f fr=%.2f  vel: fl=%.2f bl=%.2f br=%.2f fr=%.2f",
            pos_[FL], pos_[BL], pos_[BR], pos_[FR], vel_[FL], vel_[BL], vel_[BR], vel_[FR]);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type OmniWheelHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
        RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "write occured (line 229)");

        if (!serial.isDeviceOpen()){
            return hardware_interface::return_type::OK;
        }

        // send the velocity commands to the Brain, each wheel in its own named field
        v5_protocol::Packet mv{};
        mv.magic = v5_protocol::MOTOR_COMMAND_MAGIC;
        mv.front_left = static_cast<float>(cmd_[FL]);
        mv.front_right = static_cast<float>(cmd_[FR]);
        mv.back_left = static_cast<float>(cmd_[BL]);
        mv.back_right = static_cast<float>(cmd_[BR]);

        mv.intake1 = static_cast<float>(intake_cmd_[0]);
        mv.intake2 = static_cast<float>(intake_cmd_[1]);
        mv.intake3 = static_cast<float>(intake_cmd_[2]);

        serial.writeBytes(&mv, v5_protocol::PACKET_SIZE);
        
        return hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn OmniWheelHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){

        // show the magic numbers in other format so i can compare my recieved data with it
        const uint64_t pos_magic = v5_protocol::POSITIONS_MAGIC;
        const uint64_t vel_magic = v5_protocol::SPEEDS_MAGIC;
        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "what i should be getting:");
        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "GET_POS:  0x%016lX", pos_magic);
        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "GET_VEL:  0x%016lX", vel_magic);

        // convert to little endian
        const uint8_t* pos_bytes = reinterpret_cast<const uint8_t*>(&pos_magic);
        const uint8_t* vel_bytes = reinterpret_cast<const uint8_t*>(&vel_magic);

        char pos_hex[30], vel_hex[30];
        snprintf(pos_hex, sizeof(pos_hex), "%02x %02x %02x %02x %02x %02x %02x %02x",
                pos_bytes[0], pos_bytes[1], pos_bytes[2], pos_bytes[3],
                pos_bytes[4], pos_bytes[5], pos_bytes[6], pos_bytes[7]);
        snprintf(vel_hex, sizeof(vel_hex), "%02x %02x %02x %02x %02x %02x %02x %02x",
                vel_bytes[0], vel_bytes[1], vel_bytes[2], vel_bytes[3],
                vel_bytes[4], vel_bytes[5], vel_bytes[6], vel_bytes[7]);

        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "position (le):  %s", pos_hex);
        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "velocity (le):  %s", vel_hex);


        // reset parameters
        for (size_t i = 0; i < 4; ++i) {
            pos_[i] = 0.0;
            vel_[i] = 0.0;
            cmd_[i] = 0.0;
        }
        
        // reset intake params
        for (size_t i = 0; i < 3; ++i) {
            intake_pos_[i] = 0.0;
            intake_vel_[i] = 0.0;
            intake_cmd_[i] = 0.0;
        }

        // clear anything remaining in buffer
        serial.flushReceiver();
        rx_buf.clear();

        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Successfully configured!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn OmniWheelHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
        const char* SERIAL_PORT = serial_port.c_str(); // convert to char

        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Opening serial port: %s at %d baud", SERIAL_PORT, baud);
        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Packet size: %zu bytes", v5_protocol::PACKET_SIZE);

        // connection to serial port
        char errorOpening = serial.openDevice(SERIAL_PORT, baud);

        // if connection to brain fails
        if (errorOpening!=1) {
            RCLCPP_ERROR(rclcpp::get_logger("OmniWheelHardware"), "Failed to connect to device");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Successful connection to VEX Brain on port: %s\n", SERIAL_PORT);  
        
        // clear anything remaining in buffer
        serial.flushReceiver();
        rx_buf.clear();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn OmniWheelHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
        // disable the device by closing the serial connection
        serial.closeDevice();
        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Connection to VEX brain closed");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

} // namespace otto_hardware

PLUGINLIB_EXPORT_CLASS(otto_hardware::OmniWheelHardware, hardware_interface::SystemInterface)