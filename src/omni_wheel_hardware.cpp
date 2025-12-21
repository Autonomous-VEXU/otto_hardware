#include "otto_hardware/vex_comms.hpp"
#include "otto_hardware/omni_wheel_hardware.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>  

namespace otto_hardware {

hardware_interface::CallbackReturn OmniWheelHardware::on_init(hardware_interface::HardwareComponentInterfaceParams & params){
    /*
    if (info.joints.size() != 4) {
        RCLCPP_ERROR(rclcpp::get_logger("OmniWheelHardware"), "Expected 4 joints, got %zu", info.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }
    */
    RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "started");

    // if (!vex_comms.initalize("/dev/ttyTHS0")) {
    //     RCLCPP_ERROR(rclcpp::get_logger("OmniWheelHardware"), "Failed to open VEX serial");
    //     return hardware_interface::CallbackReturn::ERROR;
    // }

    std::fill(cmd_.begin(), cmd_.end(), 0.0);
    std::fill(vel_.begin(), vel_.end(), 0.0);
    std::fill(pos_.begin(), pos_.end(), 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
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
    if (!vex_comms.is_connected()) return hardware_interface::return_type::ERROR;
    for (size_t i = 0; i < 4; ++i){
        vel_[i] = cmd_[i];
        pos_[i] += cmd_[i] * period.seconds();
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
    
    return hardware_interface::return_type::OK;
}

} // namespace otto_hardware

PLUGINLIB_EXPORT_CLASS(otto_hardware::OmniWheelHardware, hardware_interface::SystemInterface)