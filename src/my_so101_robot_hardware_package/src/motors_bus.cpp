#include "my_so101_robot_hardware_package/motors_bus.hpp"


// #include "errors.hpp"
// #include <iostream>
// #define LOG_DEBUG(msg) std::cerr << "[DEBUG] " << msg << "\n"

MotorsBus::MotorsBus(
    const std::string& port,
    const std::unordered_map<std::string, Motor>& motors,
    const std::optional<std::unordered_map<std::string, MotorCalibration>> calibration
): port_(port), motors_(motors)
{
    calibration_ = setCalibration(calibration);

    for (const auto& [motor_name, motor] : motors_) {
    _id_to_name_dict[motor.id] = motor_name;
    }

    for (const auto& [motor_name, motor] : motors_) {
        _id_to_model_dict[motor.id] = motor.model;
    }

//     for (const auto& [name, motor] : motors_) {
//         _id_to_name_dict_.emplace(motor.id, name);
//     }

//     for (const auto& [model, number] : feetech::model_number_table) {
//         _model_nb_to_model_dict_.emplace(number, model);
//     }
// ￼

//     models_.reserve(motors_.size());
//     for (const auto& [name, motor] : motors_) {
//         models_.push_back(motor.model);
//     }

//     _validate_motors();
}

std::unordered_map<std::string, MotorCalibration> MotorsBus::setCalibration(const std::optional<std::unordered_map<std::string, MotorCalibration>>& calib) {
    if (calib && !calib->empty()) {
        calibration_ = *calib;
    } else {
        calibration_ = std::unordered_map<std::string, MotorCalibration>{};
    }
    return calibration_;
}

std::vector<std::string> MotorsBus::models() const {
    std::vector<std::string> result;

    result.reserve(motors_.size());

    for (const auto& [name, motor] : motors_)
    {
        result.push_back(motor.model);
    }

    return result;
}

std::string MotorsBus::_id_to_model(int motor_id) const {
    return _id_to_model_dict.at(motor_id);
}

// bool MotorsBus::is_connected() const
// {
//     return port_handler_ && port_handler_->is_open();
// }

// void MotorsBus::connect(bool handshake)
// {
//     if (is_connected()) {
//         throw DeviceAlreadyConnectedError(
//             "MotorsBus " + port_ + " is already connected. Do not call MotorsBus::connect() twice."
//         );

//     _connect(handshake);
//     set_timeout();

//     LOG_DEBUG("MotorsBus connected.");
//     }
// }

// void MotorsBus::_connect(bool handshake)
// {
//     try {
//         if (!open_port_impl()) {
//             throw std::runtime_error(
//                 "Failed to open port '" + port_ + "'."
//             );
//         }
//         else if (handshake) {
//             _handshake();
//         }
//     }
//     catch (const std::exception& /*e*/) {
//         throw ConnectionError(
//             "\nCould not connect on port '" + port_ + "'. Make sure you are using the correct port."
//             "\nTry running `python lerobot/find_port.py`\n"
//         );
//     }
// }

// std::unordered_map<std::string, std::variant<int, double>> sync_read(
//     const std::string& data_name,
//     const std::optional<std::vector<std::string>> motors = std::nullopt,
//     bool normalize = true,
//     int num_retry = 0,
//     )
// {
//     if (!is_connected()) {
//         throw DeviceNotConnectedError("MotorsBus " + port_ + " is not connected. You need to run MotorsBus::connect().")
//     }

//     _assert_protocol_is_compatible("sync_read");

//     std::vector<std::string> names = _get_motors_list(motors);
//     std::vector<int> ids;
    
//     ids.reserve(names.size());

//     std::vector<std::string> models;
//     models.reserve(names.size());

//     for (const auto& motor_name : names) {
//         const auto it = motors_.find(motor_name);
//         if (it == motors_.end()) {
//             throw std::runtime_error("Unknown motor: " + motor_name);
//         }

//         ids.push_back(it->second.id);
//         models.push_back(it->second.model);
//     }

// void _setup_sync_reader(
//     const std::vector<int>& motor_ids,
//     const int& addr,
//     const int& length)
//     {
//         sync_reader.clearParam();
//         sync_reader.start_address = addr;
//         sync_reader.data_length = length;
//         for (const auto& id_ : motor_ids) {
//             sync_reader.addParam(id_);
//         }
//     }
// }






