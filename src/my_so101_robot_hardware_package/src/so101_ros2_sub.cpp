#include "my_so101_robot_hardware_package/so101_ros2_sub.hpp"

#include <algorithm>
#include <functional>

LeRobotJointStateSubscriber::LeRobotJointStateSubscriber()
 : Node("lerobot_subscriber")
{
    // Déclaration des paramètres
    declare_parameter<std::string>("robot_name", "so101_follower");
    declare_parameter<std::string>("port", "/dev/ttyACM0");
    declare_parameter<bool>("recalibrate", false);

    // Lecture des paramètres
    robot_name_ = get_parameter("robot_name").as_string();
    port_ = get_parameter("port").as_string();
    recalibrate_ = get_parameter("recalibrate").as_bool();

    // Subscriber
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        10,
        std::bind(&LeRobotJointStateSubscriber::jointStatesCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "LeRobotController node has been started.");

    // Initialize lerobot arm
    robot_ = init_lerobot_arm();

}

std::shared_ptr<SO101> LeRobotJointStateSubscriber::init_lerobot_arm()
{
  auto robot_ = std::make_shared<SO101>(port_, robot_name_, recalibrate_);
  
  try {
    RCLCPP_INFO(this->get_logger(), "Connecting to lerobot arm...");
    robot_->connect();
    RCLCPP_INFO(this->get_logger(), "LeRobot arm connected.");
    return robot_;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to lerobot arm: %s", e.what());
    rclcpp::shutdown(); // Shutdown ROS if robot connection fails
    return nullptr;
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to lerobot arm: unknown error");
    rclcpp::shutdown();
    return nullptr;
  }
}

void LeRobotJointStateSubscriber::jointStatesCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (!robot_) {
        RCLCPP_WARN(this->get_logger(), "LeRobot arm not initialized. Skipping joint state update.");
        return;
    }

    std::map<std::string, Value> joint_states;

    const std::size_t n = std::min(msg->name.size(), msg->position.size());

    for (std::size_t i = 0; i < n; ++i) {
        const std::string & joint_name = msg->name[i];
        const double joint_value_rad = msg->position[i];

        const float joint_value_deg = static_cast<float>(
            joint_value_rad * 180.0 / 3.14159265358979323846
        );

        joint_states[joint_name] = joint_value_deg;
    }

    try {
        RCLCPP_INFO(this->get_logger(), "Sending action...");
        robot_->_bus->sync_write("Goal_Position", joint_states);
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Error sending action to lerobot arm: %s", e.what());
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LeRobotJointStateSubscriber>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}