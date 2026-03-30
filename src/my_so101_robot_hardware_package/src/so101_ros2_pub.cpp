#include "my_so101_robot_hardware_package/so101_ros2_pub.hpp"

#include <chrono>
#include <functional>

LeRobotJointStatePublisher::LeRobotJointStatePublisher()
 : Node("le_robot_joint_state_publisher")
{
    // Déclaration des paramètres
    declare_parameter<std::string>("robot_name", "so101_follower"); //so101_leader
    declare_parameter<std::string>("port", "/dev/ttyACM0");
    declare_parameter<bool>("recalibrate", false);

    // Lecture des paramètres
    robot_name_ = get_parameter("robot_name").as_string();
    port_ = get_parameter("port").as_string();
    recalibrate_ = get_parameter("recalibrate").as_bool();

    // Initialize lerobot arm
    robot_ = init_lerobot_arm();

    // Publisher sur /joint_states
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", 10);

    // Timer à 100Hz pour lire et publier les positions
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&LeRobotJointStatePublisher::publishJointStates, this)
    );

  RCLCPP_INFO(this->get_logger(), "LeRobotJointStatePublisher node has been started.");
}

std::shared_ptr<SO101> LeRobotJointStatePublisher::init_lerobot_arm()
{
  auto robot_ = std::make_shared<SO101>(port_, robot_name_, recalibrate_);
  
  try {
    RCLCPP_INFO(get_logger(), "Connecting to lerobot arm...");
    robot_->connect();
    RCLCPP_INFO(get_logger(), "LeRobot arm connected.");
    return robot_;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to connect to lerobot arm: %s", e.what());
    rclcpp::shutdown(); // Shutdown ROS if robot connection fails
    return nullptr;
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Failed to connect to lerobot arm: unknown error");
    rclcpp::shutdown();
    return nullptr;
  }
}

void LeRobotJointStatePublisher::publishJointStates()
{
  if (!robot_) {
    RCLCPP_WARN(get_logger(), "LeRobot arm not initialized. Skipping publish.");
    return;
  }

  try {
    // Lire les positions actuelles depuis le robot
    std::unordered_map<std::string, Value> positions = robot_->_bus->sync_read("Present_Position");

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "";

    for (const auto & [motor_name, value] : positions) {
      msg.name.push_back(motor_name);

      // Value = std::variant<int, float> → même type que dans le sub
      // Dans le sub tu castes float→deg, ici tu fais deg→rad
      const float value_deg = std::get<float>(value);
      const double value_rad = static_cast<double>(value_deg) * 3.14159265358979323846 / 180.0;

      msg.position.push_back(value_rad);
      msg.velocity.push_back(0.0);
      msg.effort.push_back(std::numeric_limits<double>::quiet_NaN());
    }

    publisher_->publish(msg);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error reading joint states: %s", e.what());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LeRobotJointStatePublisher>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}