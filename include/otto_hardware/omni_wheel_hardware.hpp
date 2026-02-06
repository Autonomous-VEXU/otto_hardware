#ifndef OTTO_HARDWARE__OMNI_WHEEL_HARDWARE_HPP
#define OTTO_HARDWARE__OMNI_WHEEL_HARDWARE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <array>
#include <string>
#include <vector>

namespace otto_hardware {

class OmniWheelHardware : public hardware_interface::SystemInterface
{
public:
    OmniWheelHardware() = default;

    hardware_interface::CallbackReturn on_init(hardware_interface::HardwareComponentInterfaceParams & params);

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/);
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/);
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/);

private:

    std::array<double, 4> pos_{};
    std::array<double, 4> vel_{};
    std::array<double, 4> cmd_{};
    std::array<std::string, 4> joint_names_{{
        "chassis_to_wheel_a",
        "chassis_to_wheel_b",
        "chassis_to_wheel_c",
        "chassis_to_wheel_d"
    }};

    // std::array<double, 3> intake_pos_{};
    // std::array<double, 3> intake_vel_{};
    // std::array<double, 3> intake_cmd_{};
    // std::array<std::string, 3> intake_joint_names_{{
    //     "intake_1",
    //     "intake_2",
    //     "intake_3"
    // }};

    // parameters from URDF
    int baud_rate_;
    std::string serial_port_;
};

} // namespace otto_hardware

#endif // OTTO_HARDWARE__OMNI_WHEEL_HARDWARE_HPP