#include "amr_hardware/amr_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace amr_hardware {

static const std::array<std::string, 4> JOINT_NAMES = {
    "wheel_FL_joint", "wheel_FR_joint", "wheel_RL_joint", "wheel_RR_joint"
};

hardware_interface::CallbackReturn
AMRHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    /* info_ is populated by the parent on_init */
    port_ = info_.hardware_parameters.at("serial_port");
    if (info_.hardware_parameters.count("baud_rate")) {
        baud_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
    }

    hw_cmd_.fill(0.0);
    hw_vel_.fill(0.0);
    hw_pos_.fill(0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AMRHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> si;
    for (size_t i = 0; i < 4; i++) {
        si.emplace_back(JOINT_NAMES[i], hardware_interface::HW_IF_VELOCITY, &hw_vel_[i]);
        si.emplace_back(JOINT_NAMES[i], hardware_interface::HW_IF_POSITION, &hw_pos_[i]);
    }
    return si;
}

std::vector<hardware_interface::CommandInterface>
AMRHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ci;
    for (size_t i = 0; i < 4; i++) {
        ci.emplace_back(JOINT_NAMES[i], hardware_interface::HW_IF_VELOCITY, &hw_cmd_[i]);
    }
    return ci;
}

hardware_interface::CallbackReturn
AMRHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
    if (!serial_.open(port_, baud_)) {
        RCLCPP_ERROR(rclcpp::get_logger("amr_hw"), "Cannot open serial port %s", port_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    /* 1Hz heartbeat keeps the firmware watchdog alive */
    hb_timer_ = get_node()->create_wall_timer(
        std::chrono::seconds(1),
        [this]() { serial_.send_heartbeat(); }
    );

    RCLCPP_INFO(rclcpp::get_logger("amr_hw"), "Serial port %s opened @ %d baud", port_.c_str(), baud_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
AMRHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    hb_timer_.reset();
    serial_.close();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
AMRHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    StatePacket st{};
    if (serial_.spin_once(&st)) {
        for (int i = 0; i < 4; i++) {
            /* enc_delta is cumulative over the 10ms STATE period (100Hz) */
            hw_vel_[i] = st.enc_delta[i] * RAD_PER_COUNT * 100.0;
            hw_pos_[i] += st.enc_delta[i] * RAD_PER_COUNT;
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
AMRHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    CmdVelPacket p{};
    for (int i = 0; i < 4; i++) p.omega[i] = static_cast<float>(hw_cmd_[i]);
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("amr_hw"), *get_node()->get_clock(), 1000,
        "CMD_VEL FL=%.2f FR=%.2f RL=%.2f RR=%.2f", p.omega[0], p.omega[1], p.omega[2], p.omega[3]);
    serial_.send_cmd_vel(p);
    return hardware_interface::return_type::OK;
}

}  // namespace amr_hardware

PLUGINLIB_EXPORT_CLASS(
    amr_hardware::AMRHardwareInterface,
    hardware_interface::SystemInterface)
