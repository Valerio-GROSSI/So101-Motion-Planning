#pragma once

#include <memory>
#include <map>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "my_so101_robot_hardware_package/so101.hpp"

class LeRobotJointStateSubscriber : public rclcpp::Node 
{
public:
    explicit LeRobotJointStateSubscriber();

private:
    std::shared_ptr<SO101> init_lerobot_arm();

    void jointStatesCallback(
        const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

    std::string robot_name_;
    std::string port_;
    bool recalibrate_{false};

    std::shared_ptr<SO101> robot_;
};