#include "my_so101_robot_hardware_package/so101_ros2_calib.hpp"

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

    // Initialize lerobot arm
    robot_ = init_lerobot_arm();

}

std::shared_ptr<SO101> LeRobotJointStateSubscriber::init_lerobot_arm()
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LeRobotJointStateSubscriber>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}