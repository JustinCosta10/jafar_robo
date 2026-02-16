#include "ack_control/ack_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace ack_control {

hardware_interface::CallbackReturn AckHardware::on_init(const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  hw_steer_pos_cmd_ = 0; hw_steer_pos_state_ = 0;
  hw_drive_vel_cmd_ = 0; hw_drive_vel_state_ = 0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> AckHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back("front_steer_joint", hardware_interface::HW_IF_POSITION, &hw_steer_pos_cmd_);
  command_interfaces.emplace_back("rear_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_drive_vel_cmd_);
  return command_interfaces;
}

std::vector<hardware_interface::StateInterface> AckHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back("front_steer_joint", hardware_interface::HW_IF_POSITION, &hw_steer_pos_state_);
  state_interfaces.emplace_back("rear_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_drive_vel_state_);
  return state_interfaces;
}

hardware_interface::return_type AckHardware::read(const rclcpp::Time &, const rclcpp::Duration &) {
  hw_steer_pos_state_ = hw_steer_pos_cmd_; // Loopback for testing
  hw_drive_vel_state_ = hw_drive_vel_cmd_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AckHardware::write(const rclcpp::Time &, const rclcpp::Duration &) {
  return hardware_interface::return_type::OK;
}

} // namespace ack_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ack_control::AckHardware, hardware_interface::SystemInterface)
