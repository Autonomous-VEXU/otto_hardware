//#include "otto_hardware/vex_comms.hpp"
#include "otto_hardware/omni_wheel_hardware.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>  

#include "seriallib.h"

serialib serial;

namespace otto_hardware {

    static std::string serial_port = "/dev/ttyUSB0";
    static int baud = 115200;

    struct MotorVelocities { //has to align with the brain driver 
        uint16_t magic;
        double fl;
        double fr;
        double bl;
        double br;
    };

    hardware_interface::CallbackReturn OmniWheelHardware::on_init(hardware_interface::HardwareComponentInterfaceParams & params){
        /*
        things to do:
        1. check if device is available
        2. return success message if device is found, error if not
        */

        const char* SERIAL_PORT = serial_port.c_str();

        // connection to serial port
        char errorOpening = serial.openDevice(SERIAL_PORT, baud);

        // if connection to brain fails
        if (errorOpening!=1) {
            RCLCPP_ERROR(rclcpp::get_logger("OmniWheelHardware"), "Failed to connect to device");
            return hardware_interface::CallbackReturn::ERROR; // error msg
        }
        
        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Successful connection to VEX Brain on port: %s\n", SERIAL_PORT);
        
        return hardware_interface::CallbackReturn::SUCCESS; //success msg
    }

    std::vector<hardware_interface::StateInterface> OmniWheelHardware::export_state_interfaces(){
        std::vector<hardware_interface::StateInterface> interfaces;
        for (size_t i = 0; i < 4; ++i){
            interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_[i]);
            interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_[i]);
        }
        return interfaces;
    }

    std::vector<hardware_interface::CommandInterface> OmniWheelHardware::export_command_interfaces(){
        std::vector<hardware_interface::CommandInterface> interfaces;
        for (size_t i = 0; i < 4; ++i){
            interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &cmd_[i]);
        }
        return interfaces;
    }

    hardware_interface::return_type OmniWheelHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period){

        if (serial.available() >= sizeof(MotorVelocities)){
            MotorVelocities mv;
            
            int bytes_read = serial.readBytes(&mv, sizeof(MotorVelocities));
            
            if (bytes_read == sizeof(MotorVelocities) && mv.magic == 0xFEFA){
                vel_[0] = mv.fl;
                vel_[1] = mv.fr;
                vel_[2] = mv.bl;
                vel_[3] = mv.br;
                
                for (size_t i = 0; i < 4; ++i){
                    pos_[i] += vel_[i] * period.seconds();
                }

            } else{
                RCLCPP_WARN(rclcpp::get_logger("OmniWheelHardware"), "Invalid data received");
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type OmniWheelHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
        RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "write");

        MotorVelocities mv;
        mv.magic = 0xFEFA;
        mv.fl = cmd_[0];
        mv.fr = cmd_[1];
        mv.bl = cmd_[2];
        mv.br = cmd_[3];

        serial.writeBytes(&mv, sizeof(MotorVelocities));

        return hardware_interface::return_type::OK;
    }

} // namespace otto_hardware

PLUGINLIB_EXPORT_CLASS(otto_hardware::OmniWheelHardware, hardware_interface::SystemInterface)