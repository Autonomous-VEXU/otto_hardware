#include "robot_hardware/omni_wheel_hardware.hpp"

hardware_interface::CallbackReturn
    OmniWheelHardware::on_init(const hardware_interface::HardwareInfo & info){
    if (!vex_comms.setup("/dev/ttyTHS0", 115200)) {
        RCLCPP_ERROR(rclcpp::get_logger("OmniWheelHardware"), "Failed to open VEX serial");
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;;
}

std::vector<hardware_interface::StateInterface> OmniWheelHardware::export_state_interfaces(){
    std::vector<hardware_interface::StateInterface> si;
    si.emplace_back("wheel_fl_joint", hardware_interface::HW_IF_VELOCITY, &vel_[0]);
    si.emplace_back("wheel_fr_joint", hardware_interface::HW_IF_VELOCITY, &vel_[1]);
    si.emplace_back("wheel_bl_joint", hardware_interface::HW_IF_VELOCITY, &vel_[2]);
    si.emplace_back("wheel_br_joint", hardware_interface::HW_IF_VELOCITY, &vel_[3]);
    return si;
}

std::vector<hardware_interface::CommandInterface> OmniWheelHardware::export_command_interfaces(){
    std::vector<hardware_interface::CommandInterface> ci;
    ci.emplace_back("wheel_fl_joint", hardware_interface::HW_IF_VELOCITY, &cmd_[0]);
    ci.emplace_back("wheel_fr_joint", hardware_interface::HW_IF_VELOCITY, &cmd_[1]);
    ci.emplace_back("wheel_bl_joint", hardware_interface::HW_IF_VELOCITY, &cmd_[2]);
    ci.emplace_back("wheel_br_joint", hardware_interface::HW_IF_VELOCITY, &cmd_[3]);
    return ci;
}

hardware_interface::CallbackReturn OmniWheelHardware::on_activate(const rclcpp_lifecycle::State & previous_state){ 
    // begin hardware
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmniWheelHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    // stop hardware
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type OmniWheelHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
    // eventually have a return or feedback of some sort
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type OmniWheelHardware::write(const rclcpp::Time &, const rclcpp::Duration &){
    MotorVelocities mv;
    mv.magic = 0xFEFA;
    mv.fl = cmd_[0];
    mv.fr = cmd_[1];
    mv.bl = cmd_[2];
    mv.br = cmd_[3];
    vex_comms.send_motors(mv);
    return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(OmniWheelHardware, hardware_interface::SystemInterface)