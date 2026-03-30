#pragma once

// #include "Device.hpp"
#include  <string>
#include  <filesystem>
#include  <unordered_map>
#include "my_so101_robot_hardware_package/motors_bus.hpp"
#include "my_so101_robot_hardware_package/FeetechMotorsBus.hpp"
#include  <memory>
#include  <functional>
#include "errors.hpp"
#include <iostream>
#include "my_so101_robot_hardware_package/errors.hpp"
#include <fstream>
#include <nlohmann/json.hpp>

struct MotorLimits {
    double min{};
    double max{};
};

using MotorLimitsMap = std::unordered_map<std::string, MotorLimits>;

inline const MotorLimitsMap SO101_FOLLOWER_MOTOR_LIMITS = {
    {"shoulder_pan", {- 100.0, 100.0}},
    {"shoulder_lift", {- 100.0, 100.0}},
    {"elbow_flex", {- 100.0, 100.0}},
    {"wrist_flex", {- 100.0, 100.0}},
    {"wrist_roll", {- 100.0, 100.0}},
    {"gripper", {0.0, 100.0}},
};

class SO101 // : public Device 
{
public:
    explicit SO101(
        std::string port = "/dev/ttyACM0",
        std::string name = "so101_leader",
        bool recalibrate = false
    );
    std::string port_;
    std::string name_;
    std::filesystem::path calibration_path;
    void calibrate();
    std::unordered_map<std::string, MotorCalibration> calibration;
    std::unordered_map<std::string, MotorCalibration> _load_calibration();
    std::unique_ptr < FeetechMotorsBus > _bus;
    MotorLimitsMap _motor_limits;
    bool _started{false};
    bool _reset_state{false};
    std::unordered_map < std::string, std::function < void()>> _additional_callbacks;
    void connect();
    bool is_connected() const;
    void configure();
    void _save_calibration(std::unordered_map<std::string, MotorCalibration>& calibration);
    void disconnect();



    // std:: string to_string() const;
    // int get_device_state() const ;
    // voidadd_callback(const std:: string & key, std:: function < void()> func);
    // MotorLimitsMap motor_limits() const ; // const MotorLimits& motor_limits() const;
    // disconnect void();
    // calibrate()
    // _save_calibration()
};