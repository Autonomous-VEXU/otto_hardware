#include "otto_hardware/omni_wheel_hardware.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>  
#include <iostream>
#include <vector>
#include <cstring>
#include <chrono>
#include <thread>

#include "seriallib.h"

serialib serial;

namespace otto_hardware {

    static std::string serial_port = "/dev/ttyTHS1";
    static int baud = 115200;

    static std::vector<uint8_t> rx_buf; // global buffer

    constexpr uint64_t SEND_CMD = 0xFEFAABCD1234BEEF;
    constexpr uint64_t GET_POS  = 0xF23BDEAD6789ACBD;
    constexpr uint64_t GET_VEL  = 0xF81CB00B1350C0CA;

    struct __attribute__((packed)) MotorVelocities{
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
        
        if (!serial.isDeviceOpen()){ 
            RCLCPP_ERROR(rclcpp::get_logger("OmniWheelHardware"), "Device not open, cannot read data");
            return hardware_interface::return_type::ERROR;
        }
        
        const size_t packet_size = sizeof(MotorVelocities);

        // for the timeout loop
        const auto TIMEOUT = std::chrono::milliseconds(100);
        auto start_time = std::chrono::steady_clock::now();

        // timeout loop
        while (std::chrono::steady_clock::now() - start_time < TIMEOUT) {
            // read all data available
            int available = serial.available();

            if (available > 0) {
                //std::vector<uint8_t> temp_buf(available);
                uint8_t temp_buf[128];
                size_t total_bytes_read = 0; 
                size_t bytes_read = serial.readBytes(temp_buf, sizeof(temp_buf));
                while (bytes_read > 0){
                    total_bytes_read += bytes_read;
                    rx_buf.insert(rx_buf.end(), std::begin(temp_buf), std::begin(temp_buf) + bytes_read);
                }

                if (total_bytes_read > 0) {
                    RCLCPP_DEBUG(rclcpp::get_logger("OmniWheelHardware"), "Read %ld bytes", total_bytes_read);
                    // rx_buf.insert(rx_buf.end(), temp_buf.begin(), temp_buf.begin() + bytes_read);
                }
            }

            if (rx_buf.size() > packet_size) {
                int idx = static_cast<int>(rx_buf.size()) - static_cast<int>(packet_size);
                
                while (idx >= 0) {
                    uint64_t magic; // check for magic at this pos
                    std::memcpy(&magic, rx_buf.data() + idx, sizeof(uint64_t));
                    
                    if (magic == GET_VEL || magic == GET_POS){ // got valid packet
                        MotorVelocities mv;
                        std::memcpy(&mv, rx_buf.data() + idx, packet_size);
                        
                        rx_buf.erase(rx_buf.begin(), rx_buf.begin() + idx + packet_size);
                        
                        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Got packet, magic: 0x%016lX", magic);

                        if (mv.magic == GET_VEL){
                            vel_[0] = mv.fl;
                            vel_[1] = mv.fr;
                            vel_[2] = mv.bl;
                            vel_[3] = mv.br;

                            intake_vel_[0] = mv.intake_1;
                            intake_vel_[1] = mv.intake_2;
                            intake_vel_[2] = mv.intake_3;

                            RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "vel: fl=%.2f, fr=%.2f, bl=%.2f, br=%.2f", vel_[0], vel_[1], vel_[2], vel_[3]);

                        } else if (mv.magic == GET_POS){
                            pos_[0] = mv.fl;
                            pos_[1] = mv.fr;
                            pos_[2] = mv.bl;
                            pos_[3] = mv.br;

                            intake_pos_[0] = mv.intake_1;
                            intake_pos_[1] = mv.intake_2;
                            intake_pos_[2] = mv.intake_3;

                            RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "pos: fl=%.2f, fr=%.2f, bl=%.2f, br=%.2f", pos_[0], pos_[1], pos_[2], pos_[3]);
                        }
                        return hardware_interface::return_type::OK;
                    }
                    idx--;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1 ms sleep
        }
        
        RCLCPP_WARN(rclcpp::get_logger("OmniWheelHardware"), "timout occured");
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type OmniWheelHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){

        if (!serial.isDeviceOpen()){
            return hardware_interface::return_type::OK;
        }

        // write information to command interfaces (velocity)
        MotorVelocities mv;
        mv.magic = SEND_CMD;
        mv.fl = cmd_[0];
        mv.fr = cmd_[1];
        mv.bl = cmd_[2];
        mv.br = cmd_[3];

        mv.intake_1 = intake_cmd_[0];
        mv.intake_2 = intake_cmd_[1];
        mv.intake_3 = intake_cmd_[2];

        serial.writeBytes(&mv, sizeof(MotorVelocities)); 
        
        return hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn OmniWheelHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){

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

        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Successfully configured!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn OmniWheelHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
        const char* SERIAL_PORT = serial_port.c_str(); // convert to char

        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Opening serial port: %s at %d baud", SERIAL_PORT, baud);
        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "MotorVelocities struct size: %zu bytes", sizeof(MotorVelocities));

        // connection to serial port
        char errorOpening = serial.openDevice(SERIAL_PORT, baud);

        // if connection to brain fails
        if (errorOpening!=1) {
            RCLCPP_ERROR(rclcpp::get_logger("OmniWheelHardware"), "Failed to connect to device");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Successful connection to VEX Brain on port: %s\n", SERIAL_PORT);  
        
        serial.flushReceiver();

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