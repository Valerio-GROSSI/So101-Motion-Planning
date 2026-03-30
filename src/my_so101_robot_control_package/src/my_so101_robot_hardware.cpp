#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "my_so101_robot_control_package/my_so101_robot_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_so101_robot_control_package
{

std::shared_ptr<SO101> MySo101RobotHardware::init_lerobot_arm(
  const std::string & port,
  const std::string & robot_name,
  bool recalibrate)
{
  auto robot_ = std::make_shared<SO101>(port, robot_name, recalibrate);
  try
  {
    RCLCPP_INFO(
      rclcpp::get_logger("MySo101RobotHardware"),
      "Connecting to lerobot arm...");
    robot_->connect();
    RCLCPP_INFO(
      rclcpp::get_logger("MySo101RobotHardware"),
      "LeRobot arm connected.");
    return robot_;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("MySo101RobotHardware"),
      "Failed to connect to lerobot arm: %s", e.what());
    return nullptr;   // pas de rclcpp::shutdown() ici, on laisse ros2_control gérer
  }
  catch (...)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("MySo101RobotHardware"),
      "Failed to connect: unknown error");
    return nullptr;
  }
}

hardware_interface::CallbackReturn MySo101RobotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::size_t num_joints = info_.joints.size();

  hw_states_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_states_vel_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(
    rclcpp::get_logger("MySo101RobotHardware"),
    "on_init successful. Found %zu joints.",
    num_joints);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MySo101RobotHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  robot_name = "so101_follower";
  port = "/dev/ttyACM0";
  recalibrate = false;

  robot_ = init_lerobot_arm(port, robot_name, recalibrate);
  if (!robot_)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("MySo101RobotHardware"),
      "Failed to initialize robot. Aborting configuration.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Lire les positions actuelles AVANT d'initialiser hw_commands_
  // pour éviter un saut brutal vers 0 au démarrage
  static const std::map<std::string, std::string> joint_to_motor = {
    {"joint1", "shoulder_pan"},
    {"joint2", "shoulder_lift"},
    {"joint3", "elbow_flex"},
    {"joint4", "wrist_flex"},
    {"joint5", "wrist_roll"},
    {"joint6", "gripper"}
  };

  try
  {
    auto positions = robot_->_bus->sync_read("Present_Position");

    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
      const std::string & joint_name = info_.joints[i].name;
      const std::string & motor_name = joint_to_motor.at(joint_name);

      const float value_deg = std::get<float>(positions.at(motor_name));
      const double value_rad = static_cast<double>(value_deg) * 3.14159265358979323846 / 180.0;

      // hw_commands_ = position actuelle → pas de mouvement au démarrage
      hw_states_[i]     = value_rad;
      hw_states_vel_[i] = 0.0;
      hw_commands_[i]   = value_rad;  // ← clé du trick
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("MySo101RobotHardware"),
      "Failed to read initial positions: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("MySo101RobotHardware"),
    "Hardware configured.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MySo101RobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 2);
  
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_states_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_states_vel_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MySo101RobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MySo101RobotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (std::isnan(hw_states_[i]))
    {
      hw_states_[i] = 0.0;
    }

    if (std::isnan(hw_commands_[i]))
    {
      hw_commands_[i] = hw_states_[i];
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("MySo101RobotHardware"),
    "Hardware activated.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MySo101RobotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    // Désactive le torque sur tous les moteurs
    std::map<std::string, Value> torque_off;
    torque_off["shoulder_pan"] = 0;
    torque_off["shoulder_lift"] = 0;
    torque_off["elbow_flex"] = 0;
    torque_off["wrist_flex"] = 0;
    torque_off["wrist_roll"] = 0;
    torque_off["gripper"] = 0;

    robot_->_bus->sync_write("Torque_Enable", torque_off);

    RCLCPP_INFO(
      rclcpp::get_logger("MySo101RobotHardware"),
      "Torque disabled. Robot is now free to move.");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("MySo101RobotHardware"),
      "Error disabling torque: %s", e.what());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MySo101RobotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!robot_)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("MySo101RobotHardware"),
      "Robot not initialized. Cannot read.");
    return hardware_interface::return_type::ERROR;
  }

  // Correspondance joint ROS → nom moteur bus
  // Même map que dans write()
  static const std::map<std::string, std::string> joint_to_motor = {
    {"joint1", "shoulder_pan"},
    {"joint2", "shoulder_lift"},
    {"joint3", "elbow_flex"},
    {"joint4", "wrist_flex"},
    {"joint5", "wrist_roll"},
    {"joint6", "gripper"}
  };

  try
  {
    // Lire les positions actuelles depuis le robot
    // Même appel que dans publishJointStates() du publisher
    auto positions = robot_->_bus->sync_read("Present_Position");

    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
      const std::string & joint_name = info_.joints[i].name;

      auto it = joint_to_motor.find(joint_name);
      if (it == joint_to_motor.end())
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("MySo101RobotHardware"),
          "Unknown joint: %s", joint_name.c_str());
        return hardware_interface::return_type::ERROR;
      }

      const std::string & motor_name = it->second;

      // Cherche le moteur dans les positions lues
      auto pos_it = positions.find(motor_name);
      if (pos_it == positions.end())
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("MySo101RobotHardware"),
          "Motor not found in read: %s", motor_name.c_str());
        return hardware_interface::return_type::ERROR;
      }

      // Value = std::variant<int, float> → même logique que publisher
      const float value_deg = std::get<float>(pos_it->second);
      hw_states_[i] = static_cast<double>(value_deg) * 3.14159265358979323846 / 180.0;
      hw_states_vel_[i] = 0.0;
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("MySo101RobotHardware"),
      "Error in read(): %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MySo101RobotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!robot_)
    {
        RCLCPP_ERROR(
        rclcpp::get_logger("MySo101RobotHardware"),
        "Robot not initialized. Cannot write.");
        return hardware_interface::return_type::ERROR;
    }

    static const std::map<std::string, std::string> joint_to_motor = {
      {"joint1", "shoulder_pan"},
      {"joint2", "shoulder_lift"},
      {"joint3", "elbow_flex"},
      {"joint4", "wrist_flex"},
      {"joint5", "wrist_roll"},
      {"joint6", "gripper"}
    };

    std::map<std::string, Value> joint_goals;

    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
        const std::string & joint_name = info_.joints[i].name;

        auto it = joint_to_motor.find(joint_name);
        if (it == joint_to_motor.end())
        {
          RCLCPP_ERROR(
            rclcpp::get_logger("MySo101RobotHardware"),
            "Unknown joint: %s", joint_name.c_str());
          return hardware_interface::return_type::ERROR;
        }
        
        const std::string & motor_name = it->second;
        const float joint_value_deg = static_cast<float>(
          hw_commands_[i] * 180.0 / 3.14159265358979323846
        );

        joint_goals[motor_name] = joint_value_deg;
    }

    try
    {
        robot_->_bus->sync_write("Goal_Position", joint_goals);
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(
        rclcpp::get_logger("MySo101RobotHardware"),
        "Error in write(): %s", e.what());
        return hardware_interface::return_type::ERROR;
    }

  return hardware_interface::return_type::OK;
}

}  // namespace my_so101_robot_control_package

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  my_so101_robot_control_package::MySo101RobotHardware,
  hardware_interface::SystemInterface)