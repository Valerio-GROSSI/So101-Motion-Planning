#pragma once

#ifndef MY_SO101_ROBOT_CONTROL_PACKAGE__MY_SO101_ROBOT_HARDWARE_HPP_
#define MY_SO101_ROBOT_CONTROL_PACKAGE__MY_SO101_ROBOT_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "my_so101_robot_hardware_package/so101.hpp"

namespace my_so101_robot_control_package
{

class MySo101RobotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MySo101RobotHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

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
  std::shared_ptr<SO101> init_lerobot_arm(
    const std::string & port,
    const std::string & robot_name,
    bool recalibrate);

  // ===== ROS2 control buffers =====
  std::vector<double> hw_states_;    // rad
  std::vector<double> hw_states_vel_; // rad/s
  std::vector<double> hw_commands_;  // rad

  // ===== LeRobot / SO101 =====
  std::shared_ptr<SO101> robot_;

  // ===== Config =====
  std::string robot_name;
  std::string port;
  bool recalibrate{false};
};

}  // namespace my_so101_robot_control_package

#endif  // MY_SO101_ROBOT_CONTROL_PACKAGE__MY_SO101_ROBOT_HARDWARE_HPP_