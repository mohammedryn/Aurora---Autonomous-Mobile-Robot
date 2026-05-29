#pragma once
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include "amr_hardware/serial_driver.hpp"
#include <array>
#include <chrono>

namespace amr_hardware {

class AMRHardwareInterface : public hardware_interface::SystemInterface {
public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams & params) override;

    std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    SerialDriver serial_;

    /* Wheel order: FL=0  FR=1  RL=2  RR=3 */
    std::array<double, 4> hw_cmd_{};   /* velocity commands  (rad/s) */
    std::array<double, 4> hw_vel_{};   /* velocity state     (rad/s) */
    std::array<double, 4> hw_pos_{};   /* position state     (rad)   */

    std::string port_;
    int baud_{921600};

    /* 1Hz heartbeat */
    rclcpp::TimerBase::SharedPtr hb_timer_;

    /* Encoder resolution: 7PPR × 4-edge × 19.2 gear = 537.6 counts/rev */
    static constexpr double RAD_PER_COUNT = 2.0 * M_PI / 537.6;
};

}  // namespace amr_hardware
