
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

serialib serial;

namespace otto_hardware {

    static std::string serial_port = "/dev/ttyACM1";
    static int baud = 115200;

    static std::vector<uint8_t> rx_buf; 

    constexpr uint64_t SEND_CMD = 0xFEFAABCD1234BEEF;
    constexpr uint64_t GET_POS  = 0xF23BDEAD6789ACBD;
    constexpr uint64_t GET_VEL  = 0xF81CB3924EA7C0CA;

    struct __attribute__((packed)) MotorEncoderPacket {
        uint64_t magic;
        float fl;
        float fr;
        float bl;
        float br;
    };

    struct __attribute__((packed)) MotorPowerPacket {
        uint64_t magic;
        int8_t fl;
        int8_t fr;
        int8_t bl;
        int8_t br;

        int8_t intake_1;
        int8_t intake_2;
        int8_t intake_3;
    };


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

        // Drain all available bytes from serial into rx_buf
        uint8_t temp_buf[512];
        int available;
        while ((available = serial.available()) > 0) {
            size_t bytes_read = serial.readBytes(temp_buf, std::min(available, (int)sizeof(temp_buf)));
            if (bytes_read <= 0) break;
            rx_buf.insert(rx_buf.end(), temp_buf, temp_buf + bytes_read);
        }

        const size_t packet_size = sizeof(MotorEncoderPacket);
        if (rx_buf.size() < packet_size) {
            return hardware_interface::return_type::OK;
        }

        // Scan the entire buffer for the most recent VEL and POS packets
        ssize_t last_vel_idx = -1;
        ssize_t last_pos_idx = -1;
        size_t last_packet_end = 0;

        for (size_t idx = 0; idx + packet_size <= rx_buf.size(); ++idx) {
            uint64_t magic;
            std::memcpy(&magic, rx_buf.data() + idx, sizeof(uint64_t));

            if (magic == GET_VEL) {
                last_vel_idx = idx;
                last_packet_end = idx + packet_size;
            } else if (magic == GET_POS) {
                last_pos_idx = idx;
                last_packet_end = idx + packet_size;
            }
        }

        // Extract the most recent velocity packet
        if (last_vel_idx >= 0) {
            MotorEncoderPacket mv;
            std::memcpy(&mv, rx_buf.data() + last_vel_idx, packet_size);
            vel_[0] = mv.fl;
            vel_[1] = mv.fr;
            vel_[2] = mv.bl;
            vel_[3] = mv.br;
            RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "vel: fl=%.2f, fr=%.2f, bl=%.2f, br=%.2f", vel_[0], vel_[1], vel_[2], vel_[3]);
        }

        // Extract the most recent position packet
        if (last_pos_idx >= 0) {
            MotorEncoderPacket mv;
            std::memcpy(&mv, rx_buf.data() + last_pos_idx, packet_size);
            pos_[0] = mv.fl;
            pos_[1] = mv.fr;
            pos_[2] = mv.bl;
            pos_[3] = mv.br;
            RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "pos: fl=%.2f, fr=%.2f, bl=%.2f, br=%.2f", pos_[0], pos_[1], pos_[2], pos_[3]);
        }

        // Discard everything up through the last packet we consumed.
        // Keep any trailing bytes that could be a partial packet.
        if (last_packet_end > 0) {
            rx_buf.erase(rx_buf.begin(), rx_buf.begin() + last_packet_end);
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type OmniWheelHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
        // RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "write occured (line 229)");

        if (!serial.isDeviceOpen()){
            return hardware_interface::return_type::OK;
        }

        // write information to command interfaces (velocity)
        MotorPowerPacket mv;
        mv.magic = SEND_CMD;
        // Convert rad/sec to I6F2 fixed point to save space
        mv.fl = static_cast<int8_t>(cmd_[0] * (1 << 2));
        mv.fr = static_cast<int8_t>(cmd_[1] * (1 << 2));
        mv.bl = static_cast<int8_t>(cmd_[2] * (1 << 2));
        mv.br = static_cast<int8_t>(cmd_[3] * (1 << 2));
        mv.intake_1 = static_cast<int8_t>(intake_cmd_[0] * (1 << 2));
        mv.intake_2 = static_cast<int8_t>(intake_cmd_[1] * (1 << 2));
        mv.intake_3 = static_cast<int8_t>(intake_cmd_[2] * (1 << 2));

        serial.writeBytes(&mv, sizeof(MotorPowerPacket)); 
        
        return hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn OmniWheelHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){

        // show the magic numbers in other format so i can compare my recieved data with it
        RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "what i should be getting:");
        RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "GET_POS:  0x%016lX", GET_POS);
        RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "GET_VEL:  0x%016lX", GET_VEL);

        // convert to little endian
        uint8_t* pos_bytes = (uint8_t*)&GET_POS;
        uint8_t* vel_bytes = (uint8_t*)&GET_VEL;

        char pos_hex[30], vel_hex[30];
        snprintf(pos_hex, sizeof(pos_hex), "%02x %02x %02x %02x %02x %02x %02x %02x",
                pos_bytes[0], pos_bytes[1], pos_bytes[2], pos_bytes[3],
                pos_bytes[4], pos_bytes[5], pos_bytes[6], pos_bytes[7]);
        snprintf(vel_hex, sizeof(vel_hex), "%02x %02x %02x %02x %02x %02x %02x %02x",
                vel_bytes[0], vel_bytes[1], vel_bytes[2], vel_bytes[3],
                vel_bytes[4], vel_bytes[5], vel_bytes[6], vel_bytes[7]);

        RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "position (le):  %s", pos_hex);
        RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "velocity (le):  %s", vel_hex);

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
        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "MotorPowerPacket struct size: %zu bytes", sizeof(MotorPowerPacket));
        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "MotorEncoderPacket struct size: %zu bytes", sizeof(MotorEncoderPacket));

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
