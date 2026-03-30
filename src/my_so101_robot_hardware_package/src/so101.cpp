#include "my_so101_robot_hardware_package/so101.hpp"


// #include <utility>
// #include <stdexcept>
// #include <map>


SO101::SO101(std::string port, std::string name, bool recalibrate)
: // Device() ,
 port_(std::move(port))
, name_(std::move(name))
, calibration_path(std::filesystem::path(__FILE__).parent_path() / ".cache" / (name_ + ".json")) 
, _motor_limits(SO101_FOLLOWER_MOTOR_LIMITS)
{
    if (!std::filesystem::exists(calibration_path) || recalibrate) {
        calibrate();
    }
    
    std::unordered_map<std::string, MotorCalibration> calibration = _load_calibration();

    _bus = std::make_unique<FeetechMotorsBus>(
    port_,
    std::unordered_map<std::string, Motor>{
        {"shoulder_pan",  Motor{1, "sts3215", MotorNormMode::RANGE_M100_100}},
        {"shoulder_lift", Motor{2, "sts3215", MotorNormMode::RANGE_M100_100}},
        {"elbow_flex",    Motor{3, "sts3215", MotorNormMode::RANGE_M100_100}},
        {"wrist_flex",    Motor{4, "sts3215", MotorNormMode::RANGE_M100_100}},
        {"wrist_roll",    Motor{5, "sts3215", MotorNormMode::RANGE_M100_100}},
        {"gripper",       Motor{6, "sts3215", MotorNormMode::RANGE_0_100}}},
    calibration
    );
}

void SO101::calibrate()
{
    _bus = std::make_unique<FeetechMotorsBus>(
    port_,
    std::unordered_map<std::string, Motor>{
        {"shoulder_pan",  Motor{1, "sts3215", MotorNormMode::RANGE_M100_100}},
        {"shoulder_lift", Motor{2, "sts3215", MotorNormMode::RANGE_M100_100}},
        {"elbow_flex",    Motor{3, "sts3215", MotorNormMode::RANGE_M100_100}},
        {"wrist_flex",    Motor{4, "sts3215", MotorNormMode::RANGE_M100_100}},
        {"wrist_roll",    Motor{5, "sts3215", MotorNormMode::RANGE_M100_100}},
        {"gripper",       Motor{6, "sts3215", MotorNormMode::RANGE_0_100}},
        }
    );

    connect();

    std::cout << "\n Running calibration of SO101-Leader\n";
    _bus->disable_torque();

    for (const auto& kv : _bus->motors_) {
        const std::string& motorName = kv.first; 
        _bus->write("Operating_Mode", motorName, static_cast<int>(OperatingMode::POSITION));
    }

    
    std::cout << "Move SO101-Leader to the middle of its range of motion and press ENTER...";
    std::cin.get();

    auto homing_offset = _bus->set_half_turn_homings();

    std::cout << "Move all joints sequentially through their entire ranges of motion." << std::endl;
    std::cout << "Recording positions. Press ENTER to stop..." << std::endl;

    auto [range_mins, range_maxes] = _bus->record_ranges_of_motion();

    std::unordered_map<std::string, MotorCalibration> calibration;

    for (const auto& [motor, m] : _bus->motors_)
    {
        calibration[motor] = MotorCalibration{
        m.id,
        0,
        std::get<int>(homing_offset.at(motor)),
        std::get<int>(range_mins.at(motor)),
        std::get<int>(range_maxes.at(motor))
        };
    }

    _bus->write_calibration(calibration);
    _save_calibration(calibration);

    std::cout << "Calibration saved to " << calibration_path << std::endl;

    disconnect();
}

std::unordered_map<std::string, MotorCalibration> SO101::_load_calibration()
{
    std::ifstream file(calibration_path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open calibration file: " + calibration_path.string());
    }
    nlohmann::json json_data;
    file >> json_data;

    std::unordered_map<std::string, MotorCalibration> calibration;

    for (const auto& [motor_name, motor_data] : json_data.items()) {
        calibration.emplace(
            motor_name,
            MotorCalibration{
                motor_data.at("id").get<int>(),
                motor_data.at("drive_mode").get<int>(),
                motor_data.at("homing_offset").get<int>(),
                motor_data.at("range_min").get<int>(),
                motor_data.at("range_max").get<int>(),
            }
        );
    };
    return calibration;
}

void SO101::connect() {
    if (is_connected()) {
        throw DeviceAlreadyConnectedError("SO101-Leader is already connected.");
    }
    _bus->connect();
    configure();
    std::cout << "SO101 Arm initialized with: Port=" << port_ << ", Name=" << name_ << "\n";
}

void SO101::_save_calibration(std::unordered_map<std::string, MotorCalibration>& calibration) {
    nlohmann::json save_calibration;

    for (const auto& [key, value] : calibration) {
        save_calibration[key] = {
            {"id", value.id},
            {"drive_mode", value.drive_mode},
            {"homing_offset", value.homing_offset},
            {"range_min", value.range_min},
            {"range_max", value.range_max}
        };
    }

    std::filesystem::path path(calibration_path);
    if (path.has_parent_path() && !std::filesystem::exists(path.parent_path())) {
        std::filesystem::create_directories(path.parent_path());
    }

    std::ofstream file(calibration_path);
    file << save_calibration.dump(4);
}

void SO101::disconnect() {
    if (!is_connected()){
        throw DeviceNotConnectedError("SO101-Leader is not connected.");
    }

    _bus->disconnect();

    std::cout << "SO101-Leader disconnected." << std::endl;
}

bool SO101::is_connected() const {
    return _bus && _bus->is_connected();
}

void SO101::configure() {
    _bus->disable_torque();
    _bus->configure_motors();
    for (const auto& kv : _bus->motors_) {
        const std::string& motor_name = kv.first;
        _bus->write("Operating_Mode", motor_name, static_cast<int>(OperatingMode::POSITION));
    }
}

// MotorLimitsMap SO101::_motor_limits() const {
//     return _motor_limits_;
// };

// void SO101::add_callback(const std::string& key, std::function<void()> func) {
//     _additional_callbacks[key] = std::move(func);
// }

// int SO101::get_device_state() const {
//     return _bus->sync_read("Present_Position");
// }

// void SO101::disconnect() {
//     if (!is_connected()) {
//         throw DeviceNotConnectedError("SO101-Leader is not connected.");
//     }
//     _bus_->disconnected();
//     std::cout << "SO101-Leader disconnected" << "\n";
// }

// std::string SO101::to_string() const {
//     return
//         "SO101-Leader device for SE(3) control.\n"
//         "\t----------------------------------------------\n"
//         "\tMove SO101-Leader to control SO101-Follower\n"
//         "\tIf SO101-Follower can't synchronize with SO101-Leader, "
//         "please add --recalibrate and rerun to recalibrate SO101-Leader.\n";
// }
