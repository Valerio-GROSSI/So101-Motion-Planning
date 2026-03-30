#pragma once

#include <string>
#include <memory>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "my_so101_robot_hardware_package/so101.hpp"

class LeRobotJointStatePublisher : public rclcpp::Node
{
public:
    explicit LeRobotJointStatePublisher();

private:
    std::shared_ptr<SO101> init_lerobot_arm();
    void publishJointStates();

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string robot_name_;
    std::string port_;
    bool recalibrate_{false};
    
    std::shared_ptr<SO101> robot_;
};