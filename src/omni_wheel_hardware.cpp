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

    // defaults for port config
    static std::string serial_port = "/dev/ttyACM0"; // raspberry pi output
    static int baud = 115200;

    // global buffer
    static std::vector<uint8_t> rx_buf; 

    // magic packet identifiers
    constexpr uint64_t SEND_CMD = 0xFEFAABCD1234BEEF;
    constexpr uint64_t GET_POS  = 0xF23BDEAD6789ACBD;
    constexpr uint64_t GET_VEL  = 0xF81CB00B1350C0CA;
    constexpr uint64_t RESET = 0xABCD5869B0B1CCCC;

    // MotorData packet struct
    struct __attribute__((packed)) MotorData{
        uint64_t magic;
        float fl;
        float fr;
        float bl;
        float br;

        float intake_1;
        float intake_2;
        float intake_3;
    };


    hardware_interface::CallbackReturn OmniWheelHardware::on_init(hardware_interface::HardwareComponentInterfaceParams & params){

        if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS){
            return hardware_interface::CallbackReturn::ERROR;
        }

        // initialize both intake and wheel arrays to zero
        for (size_t i = 0; i < 4; ++i) {
            pos_[i] = 0.0;
            vel_[i] = 0.0;
            cmd_[i] = 0.0;
        }

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
        
        const size_t packet_size = sizeof(MotorData);

        int available = serial.available();
        if (available > 0) {
            RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "bytes available: %d", available);
            uint8_t temp_buf[512];  // previous was 128
            size_t bytes_read = serial.readBytes(temp_buf, std::min(available, (int)sizeof(temp_buf)));
            
            if (bytes_read > 0) {
                rx_buf.insert(rx_buf.end(), std::begin(temp_buf), std::begin(temp_buf) + bytes_read);
                RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "read %zu bytes, buffer size now: %zu", bytes_read, rx_buf.size());
            }
        }

        if (rx_buf.size() >= packet_size) {
            RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "processing buffer, size: %zu", rx_buf.size());
            
            // look through everyhting in buffer
            for (size_t idx = 0; idx <= rx_buf.size() - sizeof(uint64_t); ++idx) {
                uint64_t magic;
                std::memcpy(&magic, rx_buf.data() + idx, sizeof(uint64_t));
                
                if (magic == GET_VEL || magic == GET_POS) {
                    RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "found magic: 0x%016lX *", magic);
                    
                    // get more data for packet (36 bytes im pretty sure)
                    if (idx + packet_size > rx_buf.size()) {
                        RCLCPP_WARN(rclcpp::get_logger("OmniWheelHardware"), "magic at idx %zu,  %zu more bytes for packet", 
                            idx, (idx + packet_size) - rx_buf.size());
                        break;
                    }
                    
                    // complete packet?
                    MotorData md;
                    std::memcpy(&md, rx_buf.data() + idx, packet_size);
                    
                    rx_buf.erase(rx_buf.begin(), rx_buf.begin() + idx + packet_size); // erase stuff
                    RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "removed %zu bytes from buffer, remaining: %zu", idx + packet_size, rx_buf.size());

                    if (md.magic == GET_VEL){
                        vel_[0] = md.fl;
                        vel_[1] = md.fr;
                        vel_[2] = md.bl;
                        vel_[3] = md.br;

                        intake_vel_[0] = md.intake_1;
                        intake_vel_[1] = md.intake_2;
                        intake_vel_[2] = md.intake_3;

                        RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "vel: fl=%.2f, fr=%.2f, bl=%.2f, br=%.2f", vel_[0], vel_[1], vel_[2], vel_[3]);

                    } else if (md.magic == GET_POS){
                        pos_[0] = md.fl;
                        pos_[1] = md.fr;
                        pos_[2] = md.bl;
                        pos_[3] = md.br;

                        intake_pos_[0] = md.intake_1;
                        intake_pos_[1] = md.intake_2;
                        intake_pos_[2] = md.intake_3;

                        RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "pos: fl=%.2f, fr=%.2f, bl=%.2f, br=%.2f", pos_[0], pos_[1], pos_[2], pos_[3]);
                    }
                    break; // should not return ok...
                }
            }
            
            constexpr size_t MAX_BUFFER_SIZE = packet_size * 3;
            if (rx_buf.size() > MAX_BUFFER_SIZE) {
                RCLCPP_WARN(rclcpp::get_logger("OmniWheelHardware"), "overflow %zu, no valid packet found", rx_buf.size());
                
                // mor debugging prints
                std::stringstream hex_stream;
                hex_stream << "first 36 bytes from buffer: ";
                for (size_t i = 0; i < std::min(rx_buf.size(), size_t(36)); ++i) {
                    hex_stream << std::hex << std::setw(2) << std::setfill('0') << (int)rx_buf[i] << " ";
                }
                RCLCPP_WARN(rclcpp::get_logger("OmniWheelHardware"), "%s", hex_stream.str().c_str());
                
                rx_buf.clear();
            }
        }
        
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type OmniWheelHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
        // RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "write occured (line 229)");

        if (!serial.isDeviceOpen()){
            return hardware_interface::return_type::OK;
        }

        // write information to command interfaces (velocity)
        MotorData md;
        md.magic = SEND_CMD;
        md.fl = cmd_[0];
        md.fr = cmd_[1];
        md.bl = cmd_[2];
        md.br = cmd_[3];

        md.intake_1 = intake_cmd_[0];
        md.intake_2 = intake_cmd_[1];
        md.intake_3 = intake_cmd_[2];

        serial.writeBytes(&md, sizeof(MotorData)); 
        
        return hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn OmniWheelHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){

        // reset parameters
        for (size_t i = 0; i < 4; i++) {
            pos_[i] = 0.0;
            vel_[i] = 0.0;
            cmd_[i] = 0.0;
        }
        
        // reset intake params
        for (size_t i = 0; i < 3; i++) {
            intake_pos_[i] = 0.0;
            intake_vel_[i] = 0.0;
            intake_cmd_[i] = 0.0;
        }

        // reset the odometry on the vex brain before disconnecting
        for(int i = 0; i < 4; i++ ){
            MotorData md;
            md.magic = RESET;
            md.fl = 0.0;
            md.fr = 0.0;
            md.bl = 0.0;
            md.br = 0.0;

            md.intake_1 = 0.0;
            md.intake_2 = 0.0;
            md.intake_3 = 0.0;

            // write reset cmd to vex brain
            serial.writeBytes(&md, sizeof(MotorData));
        }
        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Odometry reset command sent!");

        // clear anything remaining in buffer
        serial.flushReceiver();
        rx_buf.clear();

        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Successfully configured!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn OmniWheelHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
        const char* SERIAL_PORT = serial_port.c_str(); // convert to char

        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Opening serial port: %s at %d baud", SERIAL_PORT, baud);
        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "MotorVelocities struct size: %zu bytes", sizeof(MotorData));

        // connection to serial port for error handling
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

        // reset the odometry on the vex brain before disconnecting
        for(int i = 0; i < 4; i++ ){
            MotorData md;
            md.magic = RESET;
            md.fl = 0.0;
            md.fr = 0.0;
            md.bl = 0.0;
            md.br = 0.0;

            md.intake_1 = 0.0;
            md.intake_2 = 0.0;
            md.intake_3 = 0.0;

            // write reset cmd to vex brain
            serial.writeBytes(&md, sizeof(MotorData));
        }
        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Odometry reset command sent!");
        
        // close the connection
        serial.closeDevice();
        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Connection to VEX brain closed");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

} // namespace otto_hardware

PLUGINLIB_EXPORT_CLASS(otto_hardware::OmniWheelHardware, hardware_interface::SystemInterface)