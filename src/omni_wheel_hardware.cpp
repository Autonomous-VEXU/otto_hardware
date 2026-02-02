#include "otto_hardware/omni_wheel_hardware.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>  
#include <iostream>

#include "seriallib.h"

serialib serial;

namespace otto_hardware {

    static std::string serial_port = "/dev/ttyACM1";
    static int baud = 115200;

    struct __attribute__((packed)) MotorVelocities{
        uint16_t magic;
        double fl;
        double fr;
        double bl;
        double br;

        double intake_1;
        double intake_2;
        double intake_3;
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

    hardware_interface::return_type OmniWheelHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period){
        
        if (!serial.isDeviceOpen()){
            return hardware_interface::return_type::OK;
        }
        
        int available = serial.available();

        // updates the state interfaces (position and velocity)
        if (available >= static_cast<int>(sizeof(MotorVelocities))){
            MotorVelocities mv;
            
            int bytes_read = serial.readBytes(&mv, sizeof(MotorVelocities));
            
            if (bytes_read == static_cast<int>(sizeof(MotorVelocities)) && mv.magic == 0xF81C){
                vel_[0] = mv.fl;
                vel_[1] = mv.fr;
                vel_[2] = mv.bl;
                vel_[3] = mv.br;

                intake_vel_[0] = mv.intake_1;
                intake_vel_[1] = mv.intake_2;
                intake_vel_[2] = mv.intake_3;
                
            }else if(bytes_read = static_cast<int>(sizeof(MotorVelocities)) && mv.magic == 0xF23B){
                pos_[0] = mv.fl;
                pos_[1] = mv.fl;
                pos_[2] = mv.fl;
                pos_[3] = mv.fl;

                intake_pos_[0] = mv.intake_1;
                intake_pos_[1] = mv.intake_2;
                intake_pos_[2] = mv.intake_3;
            }
        }
        
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type OmniWheelHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){

      
        if (!serial.isDeviceOpen()){
            return hardware_interface::return_type::OK;
        }

        // write information to command interfaces (velocity)
        MotorVelocities mv;
        mv.magic = 0xFEFA;
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
        //reset intake params
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