#include "my_so101_robot_hardware_package/FeetechMotorsBus.hpp"

#define LOG_DEBUG(msg) std::cerr << "[DEBUG] " << msg << "\n"

std::vector<uint8_t> _split_into_byte_chunks(int value, int length) {
    std::vector<uint8_t> data;

    if (length == 1)
    {
        data = {value};
    }
    else if (length == 2)
    {
        data = {
            scservo_def::SCS_LOBYTE(value),
            scservo_def::SCS_HIBYTE(value)
        };
    }
    else if (length == 4)
    {
        data = {
            scservo_def::SCS_LOBYTE(scservo_def::SCS_LOWORD(value)),
            scservo_def::SCS_HIBYTE(scservo_def::SCS_LOWORD(value)),
            scservo_def::SCS_LOBYTE(scservo_def::SCS_HIWORD(value)),
            scservo_def::SCS_HIBYTE(scservo_def::SCS_HIWORD(value))
        };
    }

    return data;
}

void patch_setPacketTimeout(int packet_length) {
        // packet_start_time = getCurrentTime();
        // packet_timeout = (tx_time_per_byte * static_cast<double>(packet_length)) + (tx_time_per_byte * 3.0) + 50.0;
}

const bool FeetechMotorsBus::apply_drive_mode = true;
const std::vector<int> FeetechMotorsBus::available_baudrates = tables::SCAN_BAUDRATES;
const int FeetechMotorsBus::default_baudrate = DEFAULT_BAUDRATE;
const int FeetechMotorsBus::default_timeout = DEFAULT_TIMEOUT_MS;
const tables::ModelBaudrateTable FeetechMotorsBus::model_baudrate_table = tables::MODEL_BAUDRATE_TABLE;
const tables::ModelControlTable& FeetechMotorsBus::model_ctrl_table = tables::MODEL_CONTROL_TABLE;
const tables::ModelEncodingTable& FeetechMotorsBus::model_encoding_table = tables::MODEL_ENCODING_TABLE;
const tables::StringIntMap& FeetechMotorsBus::model_number_table = tables::MODEL_NUMBER_TABLE;
const tables::StringIntMap& FeetechMotorsBus::model_resolution_table = tables::MODEL_RESOLUTION;
const std::unordered_set<std::string> FeetechMotorsBus::normalized_data = NORMALIZED_DATA;

FeetechMotorsBus::FeetechMotorsBus(
    const std::string& port,
    const std::unordered_map<std::string, Motor>& motors,
    const std::optional<std::unordered_map<std::string, MotorCalibration>> calibration,
    int protocol_version
)
: MotorsBus(port, motors, calibration)
, protocol_version_(protocol_version)
, port_handler(port_)
, packet_handler(PacketHandler(protocol_version))
, sync_reader(&port_handler, &packet_handler, 0, 0)
, sync_writer(&port_handler, &packet_handler, 0, 0)
{
    _assert_same_protocol();

    for (const auto& model : models())
    {
        if (tables::MODEL_PROTOCOL.at(model) != protocol_version_)
        {
            throw std::runtime_error("Some motors are incompatible with protocol_version=" + std::to_string(protocol_version_));
        }
    }

    // for (const auto& [motor_name, motor] : motors_) {
    //     _id_to_name_dict[motor.id] = motor_name;
    // }

    // port_handler_ = std::make_unique<PortHandler>(port_);
    

    // // // patch_setPacketTimeout(port_handler_);
    // packet_handler_ = PacketHandler(protocol_version);
    // port_handler_ = PortHandler(port_);
    // // sync_reader_ = GroupSyncRead(port_handler_, packet_handler_, 0, 0);
    // // sync_writer_ = GroupSyncWrite(port_handler_, packet_handler_, 0, 0);
    // // _comm_success = scservo_def::COMM_SUCCESS;
    // // _no_error = 0x00;

    // // for (const auto& model : models_) {
    // // if (feetech::MODEL_PROTOCOL.at(model) != protocol_version_) {
    // //     throw std::invalid_argument("Some motors are incompatible with protocol_version=" + std::to_string(protocol_version_))
    // // }
    // // }


}

void FeetechMotorsBus::_assert_same_protocol()
{
    for (const auto& model : models())
    {
        if (tables::MODEL_PROTOCOL.at(model) != protocol_version_)
        {
            throw std::runtime_error("Some motors use an incompatible protocol.");
        }
    }
}

void FeetechMotorsBus::connect(bool handshake)
{
    if (is_connected()) {
        throw DeviceAlreadyConnectedError("FeetechMotorsBus " + port_ + " is already connected. Do not call FeetechMotorsBus::connect() twice.");
    }

    _connect(handshake);
    set_timeout();
    LOG_DEBUG("FeetechMotorsBus connected.");
}

bool FeetechMotorsBus::is_connected() const {
    // """bool: `True` if the underlying serial port is open."""
    return port_handler.is_open;
}

void FeetechMotorsBus::_connect(bool handshake)
{
    std::cerr << "Opening port..." << std::endl;

    if (!port_handler.openPort()) {
        std::cerr << "openPort() failed" << std::endl;
        throw std::runtime_error("Failed to open port '" + port_ + "'.");
    }

    std::cerr << "Port opened OK" << std::endl;

    if (handshake) {
        std::cerr << "Starting handshake..." << std::endl;
        _handshake();
        std::cerr << "Handshake OK" << std::endl;
    }
}

void FeetechMotorsBus::set_timeout(std::optional<int> timeout_ms)
{
    int timeout = timeout_ms.has_value() ? *timeout_ms : default_timeout;
    port_handler.setPacketTimeoutMillis(timeout);
}

void FeetechMotorsBus::_handshake() {
    _assert_motors_exist();
    _assert_same_firmware();
}

void FeetechMotorsBus::_assert_motors_exist()
{
    if (motors_.empty()) {
        throw std::runtime_error("motors_ is empty");
    }

    std::unordered_map<int, int> expected_models;

    for (const auto& [ignored, motor] : motors_) {
        std::cout << "motor.id = " << motor.id << ", motor.model = [" << motor.model << "]\n";

        auto it = tables::MODEL_NUMBER_TABLE.find(motor.model);
        if (it == tables::MODEL_NUMBER_TABLE.end()) {
            throw std::runtime_error("Unknown motor model: " + motor.model);
        }

        expected_models[motor.id] = it->second;
    }

    std::unordered_map<int, int> found_models;

    for (int id : ids()) {
        std::cout << id << '\n';
        auto model_nb = ping(id);
        if (model_nb.has_value()) {
            found_models[id] = model_nb.value();
        }
    }

    std::vector<int> missing_ids;
    for (int id : ids()) {
        if (found_models.find(id) == found_models.end()) {
            missing_ids.push_back(id);
        }
    }

    std::unordered_map<int, std::pair<int, int>> wrong_models;
    for (const auto& [id, found] : found_models) {
        auto it = expected_models.find(id);
        if (it != expected_models.end() && it->second != found) {
            wrong_models[id] = {it->second, found};
        }
    }

    if (!missing_ids.empty() || !wrong_models.empty()) {
        std::ostringstream error;
        error << "motor check failed on port '" << port_ << "':\n";

        if (!missing_ids.empty()) {
            error << "\nMissing motor IDs:\n";
            for (int id : missing_ids) {
                auto it = expected_models.find(id);
                error << "  - " << id << " (expected model: ";
                error << (it != expected_models.end() ? std::to_string(it->second) : "unknown");
                error << ")\n";
            }
        }

        if (!wrong_models.empty()) {
            error << "\nMotors with incorrect model numbers:\n";
            for (const auto& [id, models] : wrong_models) {
                error << "  - " << id
                      << " (" << _id_to_name(id) << "): expected "
                      << models.first << ", found "
                      << models.second << "\n";
            }
        }

        throw std::runtime_error(error.str());
    }
}

void FeetechMotorsBus::_assert_same_firmware() {
    const std::unordered_map<int, std::string> firmware_versions = _read_firmware_version(ids(), true);

    std::unordered_set<std::string> unique_versions;

    for (const auto& [id, version] : firmware_versions)
    {
        unique_versions.insert(version);
    }

    if (unique_versions.size() != 1) 
    {
        std::ostringstream msg;
        msg << "Some Motors use different firmware versions:\n";
        
        for (const auto& [id, version] : firmware_versions)
        {
            msg << "id=" << id << " -> " << version << "\n";
        }
        
        msg << "\nUpdate their firmware first using Feetech's software.\n"
            << "Visit https://www.feetechrc.com/software.";

        throw std::runtime_error(msg.str());
    }
}

std::vector<int> FeetechMotorsBus::ids() const
{
    std::vector<int> result;

    for (const auto& [ignored, motor] : motors_)
        result.push_back(motor.id);

    return result;
}

std::optional<int> FeetechMotorsBus::ping(NameOrID motor, int num_retry, bool raise_on_error)
{
    int id_ = _get_motor_id(motor);

    int model_number = 0;
    int comm = 0;
    int error = 0;

    for (int n_try = 0; n_try < 1 + num_retry; ++n_try)
    {
        std::tie(model_number, comm, error) = packet_handler.ping(port_handler, id_);
        
        if (_is_comm_success(comm))
            break;

        LOG_DEBUG("ping failed for id=" << id_
                  << " n_try=" << n_try
                  << " comm=" << comm
                  << " error=" << error);
    }

    if (!_is_comm_success(comm))
    {
        if (raise_on_error)
            throw std::runtime_error(packet_handler.getTxRxResult(comm));
        else
            return std::nullopt;
    }

    if (_is_error(error))
    {
        if (raise_on_error)
            throw std::runtime_error(packet_handler.getRxPacketError(error));
        else
            return std::nullopt;
    }

    return model_number;
}

std::string FeetechMotorsBus::_id_to_name(int motor_id) {
    return _id_to_name_dict[motor_id];
}

int FeetechMotorsBus::_get_motor_id(const NameOrID& motor)
{
    if (std::holds_alternative<std::string>(motor))
    {
        const std::string& name = std::get<std::string>(motor);
        return motors_.at(name).id;
    }
    else if (std::holds_alternative<int>(motor))
    {
        return std::get<int>(motor);
    }
    else
    {
        throw std::invalid_argument("motor should be int or string");
    }
}

bool FeetechMotorsBus::_is_comm_success(int comm) const {
    return comm == _comm_success;
}

bool FeetechMotorsBus::_is_error(int error) const {
    return error != _no_error;
}

std::unordered_map<int, std::string> FeetechMotorsBus::_read_firmware_version(const std::vector<int>& motor_ids, bool raise_on_error) {
    std::unordered_map<int, std::string> firmware_versions;

    for (int id_ : motor_ids)
    {
        int firm_ver_major, comm, error;
        std::tie(firm_ver_major, comm, error) = _read(tables::FIRMWARE_MAJOR_VERSION.address, tables::FIRMWARE_MAJOR_VERSION.size, id_, raise_on_error);

        if (!_is_comm_success(comm) || _is_error(error))
            continue;

        int firm_ver_minor;
        std::tie(firm_ver_minor, comm, error) = _read(tables::FIRMWARE_MINOR_VERSION.address, tables::FIRMWARE_MINOR_VERSION.size, id_, raise_on_error);

        if (!_is_comm_success(comm) || _is_error(error))
            continue;

        firmware_versions[id_] = std::to_string(firm_ver_major) + "." + std::to_string(firm_ver_minor);
    }

    return firmware_versions;
}

std::tuple<int,int,int> FeetechMotorsBus::_read(int address, int length, int motor_id, int num_retry, bool raise_on_error, const std::string& err_msg) {
    int value = 0;
    int comm = 0;
    int error = 0;

    for (int n_try = 0; n_try < 1 + num_retry; ++n_try)
    {
        if (length == 1)
        {
            std::tie(value, comm, error) = packet_handler.read1ByteTxRx(&port_handler, motor_id, address);
        }
        else if (length == 2)
        {
            std::tie(value, comm, error) = packet_handler.read2ByteTxRx(&port_handler, motor_id, address);
        }
        else if (length == 4)
        {
            std::tie(value, comm, error) = packet_handler.read4ByteTxRx(&port_handler, motor_id, address);
        }
        else
        {
            throw std::invalid_argument("Unsupported length");
        }

        if (_is_comm_success(comm))
            break;

        LOG_DEBUG("Failed to read @address=" << address
                  << " (length=" << length << ") on motor_id=" << motor_id
                  << " (n_try=" << n_try << "): "
                  << packet_handler.getTxRxResult(comm)
                );
    }

    if (!_is_comm_success(comm) && raise_on_error)
    {
        throw ConnectionError(
            err_msg + " " + packet_handler.getTxRxResult(comm)
        );
    }
    else if (_is_error(error) && raise_on_error)
    {
        throw std::runtime_error(
            err_msg + " " + packet_handler.getRxPacketError(error)
        );
    }

    return {value, comm, error};
}

void FeetechMotorsBus::disable_torque(const std::vector<std::string>& motors, int num_retry)
{
    auto motors_list = _get_motors_list(motors);

    for (const auto& motor : motors_list)
    {
        // ? true
        write("Torque_Enable", motor, static_cast<int>(TorqueMode::DISABLED), true, num_retry);
        write("Lock", motor, 0, true, num_retry);
    }
}

std::vector<std::string> FeetechMotorsBus::_get_motors_list(const std::optional<std::variant<std::string, std::vector<std::string>>>& motors)
{
    if (!motors)
    {
        std::vector<std::string> result;
        result.reserve(motors_.size());

        for (const auto& [name, _] : motors_)
            result.push_back(name);

        return result;
    }

    if (std::holds_alternative<std::string>(*motors))
    {
        return {std::get<std::string>(*motors)};
    }

    if (std::holds_alternative<std::vector<std::string>>(*motors))
    {
        return std::get<std::vector<std::string>>(*motors);
    }

    throw std::invalid_argument("Invalid motors argument");
}

void FeetechMotorsBus::write(const std::string& data_name, const std::string& motor, Value value, bool normalize, int num_retry)
{
    if (!is_connected())
    {
        throw DeviceNotConnectedError(
            "FeetechMotorsBus ('" + port_ + "') is not connected. "
            "You need to run FeetechMotorsBus::connect()."
        );
    }

    int id_ = motors_.at(motor).id;
    std::string model = motors_.at(motor).model;

    auto [addr, length] = get_address(model_ctrl_table, model, data_name);

    if (normalize && normalized_data.count(data_name))
    {
        double val = std::visit([](auto v){ return static_cast<double>(v); }, value);
        std::map<int,double> tmp{{id_, val}};
        value = _unnormalize(tmp)[id_];
    }

    {
        int raw = std::visit([](auto v){ return static_cast<int>(v); }, value);
        std::map<int,int> tmp{{id_, raw}};
        value = _encode_sign(data_name, tmp)[id_];
    }

    int raw = std::visit([](auto v){ return static_cast<int>(v); }, value);

    std::string err_msg =
        "Failed to write '" + data_name +
        "' on id=" + std::to_string(id_) +
        " with '" + std::to_string(raw) +
        "' after " + std::to_string(num_retry + 1) + " tries.";

    _write(addr, length, id_, raw, num_retry, true, err_msg);
}

std::pair<int,int> FeetechMotorsBus::get_address(const tables::ModelControlTable& model_ctrl_table, const std::string& model, const std::string& data_name)
{

    const auto& ctrl_table = get_ctrl_table(model_ctrl_table, model);

    auto jt = ctrl_table.find(data_name);

    if (jt == ctrl_table.end())
        throw std::runtime_error("Address for '" + data_name + "' not found.");

    return {jt->second.address, jt->second.size};
}
// {
//     auto ctrl_table = get_ctrl_table(model_ctrl_table, model);

//     auto it = ctrl_table.find(data_name);

//     if (it == ctrl_table.end())
//     {
//         throw std::runtime_error(
//             "Address for '" + data_name + "' not found in " + model + " control table."
//         );
//     }

//     return it->second;
// }

const tables::ControlTable& FeetechMotorsBus::get_ctrl_table(const tables::ModelControlTable& model_ctrl_table, const std::string& model) const
{
    auto it = model_ctrl_table.find(model);

    if (it == model_ctrl_table.end())
    {
        throw std::runtime_error("Control table for model=" + model + " not found.");
    }

    return *(it->second);
}

std::map<int,int> FeetechMotorsBus::_unnormalize(const std::map<int,double>& ids_values)
{
    if (calibration_.empty())    
    {
        throw std::runtime_error("No calibration registered.");
    }

    std::map<int,int> unnormalized_values;

    for (const auto& [id_, val_in] : ids_values)
    {
        double val = val_in;

        std::string motor = _id_to_name(id_);

        int min_ = calibration_[motor].range_min;
        int max_ = calibration_[motor].range_max;

        bool drive_mode = apply_drive_mode && calibration_[motor].drive_mode;

        if (max_ == min_)
        {
            throw std::runtime_error(
                "Invalid calibration for motor '" + motor +
                "': min and max are equal."
            );
        }

        auto norm_mode = motors_[motor].norm_mode;

        if (norm_mode == MotorNormMode::RANGE_M100_100)
        {
            val = drive_mode ? -val : val;

            double bounded_val = std::min(100.0, std::max(-100.0, val));

            unnormalized_values[id_] =
                static_cast<int>(((bounded_val + 100) / 200) * (max_ - min_) + min_);
        }
        else if (norm_mode == MotorNormMode::RANGE_0_100)
        {
            val = drive_mode ? (100 - val) : val;

            double bounded_val = std::min(100.0, std::max(0.0, val));

            unnormalized_values[id_] =
                static_cast<int>((bounded_val / 100) * (max_ - min_) + min_);
        }
        else if (norm_mode == MotorNormMode::DEGREES)
        {
            double mid = (min_ + max_) / 2.0;

            int max_res = model_resolution_table.at(_id_to_model(id_)) - 1;

            unnormalized_values[id_] =
                static_cast<int>((val * max_res / 360.0) + mid);
        }
        else
        {
            throw std::runtime_error("Normalization mode not implemented.");
        }
    }

    return unnormalized_values;
}

std::map<int,int> FeetechMotorsBus::_encode_sign(const std::string& data_name, std::map<int,int> ids_values)
{
    for (auto& [id_, value] : ids_values)
    {
        std::string model = _id_to_model(id_);

        auto it = model_encoding_table.find(model);

        if (it != model_encoding_table.end())
        {
            const auto& encoding_table = it->second;

        auto jt = encoding_table->find(data_name);

        if (jt != encoding_table->end())
            {
                int sign_bit = jt->second;
                value = encode_sign_magnitude(value, sign_bit);
            }
        }
    }

    return ids_values;
}

int FeetechMotorsBus::encode_sign_magnitude(int value, int sign_bit_index)
{
    int max_magnitude = (1 << sign_bit_index) - 1;

    int magnitude = std::abs(value);

    if (magnitude > max_magnitude)
    {
        throw std::runtime_error(
            "Magnitude " + std::to_string(magnitude) +
            " exceeds " + std::to_string(max_magnitude) +
            " (max for sign_bit_index=" + std::to_string(sign_bit_index) + ")"
        );
    }

    int direction_bit = (value < 0) ? 1 : 0;

    return (direction_bit << sign_bit_index) | magnitude;
}

std::pair<int,int> FeetechMotorsBus::_write(int addr, int length, int motor_id, int value, int num_retry, bool raise_on_error, const std::string& err_msg)
{
    std::vector<uint8_t> data = _serialize_data(value, length);

    int comm = 0;
    int error = 0;

    for (int n_try = 0; n_try <= num_retry; ++n_try)
    {
        std::tie(comm, error) =
            packet_handler.writeTxRx(&port_handler, motor_id, addr, length, data);

        if (_is_comm_success(comm))
            break;

        std::cerr
            << "Failed to sync write @addr=" << addr
            << " (length=" << length << ")"
            << " on id=" << motor_id
            << " with value=" << value
            << " (try=" << n_try << "): "
            << packet_handler.getTxRxResult(comm)
            << std::endl;
    }

    if (!_is_comm_success(comm) && raise_on_error)
    {
        throw std::runtime_error(
            err_msg + " " + packet_handler.getTxRxResult(comm)
        );
    }
    else if (_is_error(error) && raise_on_error)
    {
        throw std::runtime_error(
            err_msg + " " + packet_handler.getRxPacketError(error)
        );
    }

    return {comm, error};
}

std::vector<uint8_t> FeetechMotorsBus::_serialize_data(int value, int length)
{
    if (value < 0)
    {
        throw std::invalid_argument("Negative values are not allowed: " + std::to_string(value));
    }

    uint32_t max_value;

    switch (length)
    {
        case 1: max_value = 0xFF; break;
        case 2: max_value = 0xFFFF; break;
        case 4: max_value = 0xFFFFFFFF; break;
        default:
            throw std::runtime_error("Unsupported byte size: " + std::to_string(length) + ". Expected [1, 2, 4].");
    }

    if (static_cast<uint32_t>(value) > max_value)
    {
        throw std::runtime_error(
            "Value " + std::to_string(value) +
            " exceeds the maximum for " +
            std::to_string(length) + " bytes (" +
            std::to_string(max_value) + ")."
        );
    }

    return _split_into_byte_chunks(value, length);
}

void FeetechMotorsBus::configure_motors()
{
    for (const auto& [motor, _] : motors_)
    {
        // Reduce response delay to minimum (2µs)
        write("Return_Delay_Time", motor, 0);

        // Increase acceleration limits
        write("Maximum_Acceleration", motor, 254);
        write("Acceleration", motor, 254);
    }
}

std::unordered_map<NameOrID, Value>FeetechMotorsBus::set_half_turn_homings(const std::optional<std::variant<NameOrID, std::vector<NameOrID>>>& motors)
{
    std::vector<NameOrID> motors_list;

    if (!motors.has_value())
    {
        for (const auto& [name, _] : motors_)
            motors_list.push_back(name);
    }
    else if (std::holds_alternative<NameOrID>(*motors))
    {
        motors_list.push_back(std::get<NameOrID>(*motors));
    }
    else if (std::holds_alternative<std::vector<NameOrID>>(*motors))
    {
        motors_list = std::get<std::vector<NameOrID>>(*motors);
    }
    else
    {
        throw std::invalid_argument("Invalid motors argument");
    }

    reset_calibration(motors_list);

    std::vector<std::string> motor_names;
    motor_names.reserve(motors_list.size());

    for (const auto& m : motors_list)
    {
        if (std::holds_alternative<std::string>(m))
            motor_names.push_back(std::get<std::string>(m));
        else
            motor_names.push_back(_id_to_name(std::get<int>(m)));
    }

    auto actual_positions =
        sync_read("Present_Position", motor_names, false);

    auto raw_offsets =
        _get_half_turn_homings(actual_positions);

    std::unordered_map<NameOrID, Value> homing_offsets;

    for (const auto& [motor, offset] : raw_offsets)
    {
        homing_offsets[NameOrID(motor)] = offset;
    }

    for (const auto& [motor, offset] : homing_offsets)
    {
        std::string name =
            std::holds_alternative<std::string>(motor)
            ? std::get<std::string>(motor)
            : _id_to_name(std::get<int>(motor));

        write("Homing_Offset", name, offset);
    }

    return homing_offsets;
}

void FeetechMotorsBus::reset_calibration(const std::optional<std::variant<NameOrID, std::vector<NameOrID>>>& motors)
{
    std::vector<NameOrID> motors_list;

    if (!motors.has_value())
    {
        for (const auto& [name, _] : motors_)
            motors_list.push_back(name);
    }
    else if (std::holds_alternative<NameOrID>(*motors))
    {
        motors_list.push_back(std::get<NameOrID>(*motors));
    }
    else if (std::holds_alternative<std::vector<NameOrID>>(*motors))
    {
        motors_list = std::get<std::vector<NameOrID>>(*motors);
    }
    else
    {
        throw std::invalid_argument("Invalid motors argument");
    }

    for (const auto& motor : motors_list)
    {
        std::string model = _get_motor_model(motor);
        
        int max_res = model_resolution_table.at(model) - 1;

        std::string motor_name;

        if (std::holds_alternative<std::string>(motor))
            motor_name = std::get<std::string>(motor);
        else
            motor_name = _id_to_name(std::get<int>(motor));

        write("Homing_Offset", motor_name, 0, false);
        write("Min_Position_Limit", motor_name, 0, false);
        write("Max_Position_Limit", motor_name, max_res, false);
    }

    calibration_.clear();
}

std::unordered_map<std::string, Value> FeetechMotorsBus::sync_read(
    const std::string& data_name,
    const std::optional<std::vector<std::string>>& motors,
    bool normalize,
    int num_retry)
{
    if (!is_connected())
    {
        throw DeviceNotConnectedError(
            "FeetechMotorsBus('" + port_ + "') is not connected. "
            "You need to run FeetechMotorsBus::connect()."
        );
    }

    _assert_protocol_is_compatible("sync_read");

    // ----- get motor names -----
    std::vector<std::string> names = _get_motors_list(motors);

    std::vector<int> ids;
    std::vector<std::string> models;

    for (const auto& motor : names)
    {
        ids.push_back(motors_.at(motor).id);
        models.push_back(motors_.at(motor).model);
    }

    // optional safety check (same as Python)
    if (_has_different_ctrl_tables())
    {
        assert_same_address(model_ctrl_table, models, data_name);
    }

    std::string model = models.front();

    auto [addr, length] = get_address(model_ctrl_table, model, data_name);

    std::string err_msg =
        "Failed to sync read '" + data_name +
        "' after " + std::to_string(num_retry + 1) + " tries.";

    auto [ids_values, _] =
        _sync_read(addr, length, ids, num_retry, true, err_msg);

    ids_values = _decode_sign(data_name, ids_values);

    std::unordered_map<std::string, Value> result;

    if (normalize && normalized_data.count(data_name))
    {
        auto normalized_values = _normalize(ids_values);

        for (const auto& [id, value] : normalized_values)
        {
            result[_id_to_name(id)] = value;
        }
    }
    else
    {
        for (const auto& [id, value] : ids_values)
        {
            result[_id_to_name(id)] = value;
        }
    }

    return result;
}

std::unordered_map<std::string, Value> FeetechMotorsBus::_get_half_turn_homings(
    const std::unordered_map<std::string, Value>& positions)
{
    std::unordered_map<std::string, Value> half_turn_homings;

    for (const auto& [motor, pos_val] : positions)
    {
        std::string model = _get_motor_model(motor);

        int max_res = model_resolution_table.at(model) - 1;

        int pos = std::visit(
            [](auto v) { return static_cast<int>(v); },
            pos_val
        );

        half_turn_homings[motor] = pos - (max_res / 2);
    }

    return half_turn_homings;
}

void FeetechMotorsBus::_assert_protocol_is_compatible(const std::string& instruction_name)
    {
        if (instruction_name == "sync_read" && protocol_version_ == 1)
        {
            throw std::logic_error("'Sync Read' is not available with Feetech motors using Protocol 1. Use 'Read' sequentially instead.");
        }
        
        if (instruction_name == "broadcast_ping" && protocol_version_ == 1)
        {
            throw std::logic_error("'Broadcast Ping' is not available with Feetech motors using Protocol 1. Use 'Ping' sequentially instead.");
        }
    }

void FeetechMotorsBus::assert_same_address(
    const tables::ModelControlTable& model_ctrl_table,
    const std::vector<std::string>& motor_models,
    const std::string& data_name)
{
    std::vector<int> all_addr;
    std::vector<int> all_bytes;

    for (const auto& model : motor_models)
    {
        auto [addr, bytes] = get_address(model_ctrl_table, model, data_name);
        all_addr.push_back(addr);
        all_bytes.push_back(bytes);
    }

    std::set<int> addr_set(all_addr.begin(), all_addr.end());
    if (addr_set.size() != 1)
    {
        throw std::runtime_error(
            "At least two motor models use a different address for data_name='" +
            data_name + "'"
        );
    }

    std::set<int> bytes_set(all_bytes.begin(), all_bytes.end());
    if (bytes_set.size() != 1)
    {
        throw std::runtime_error(
            "At least two motor models use a different bytes representation for data_name='" +
            data_name + "'"
        );
    }
}

std::pair<std::map<int,int>, int> FeetechMotorsBus::_sync_read(
    int addr,
    int length,
    const std::vector<int>& motor_ids,
    int num_retry,
    bool raise_on_error,
    const std::string& err_msg)
{
    _setup_sync_reader(motor_ids, addr, length);

    int comm = 0;

    for (int n_try = 0; n_try <= num_retry; ++n_try)
    {
        comm = sync_reader.txRxPacket();

        if (_is_comm_success(comm))
            break;

        std::cerr << "Failed to sync read @addr=" << addr
                  << " (length=" << length << ") on motors "
                  << " (try=" << n_try << "): "
                  << packet_handler.getTxRxResult(comm)
                  << std::endl;
    }

    if (!_is_comm_success(comm) && raise_on_error)
    {
        throw std::runtime_error(
            err_msg + " " + packet_handler.getTxRxResult(comm)
        );
    }

    std::map<int,int> values;

    for (int id : motor_ids)
    {
        values[id] = sync_reader.getData(id, addr, length);
    }

    return {values, comm};
}

std::map<int,int> FeetechMotorsBus::_decode_sign(const std::string& data_name, std::map<int,int> ids_values)
{
    for (auto& [id, value] : ids_values)
    {
        std::string model = _id_to_model(id);

        auto it = model_encoding_table.find(model);

        if (it != model_encoding_table.end())
        {
            const auto* encoding_table = it->second;

            auto jt = encoding_table->find(data_name);

            if (jt != encoding_table->end())
            {
                int sign_bit = jt->second;
                value = decode_sign_magnitude(value, sign_bit);
            }
        }
    }

    return ids_values;
}

int FeetechMotorsBus::decode_sign_magnitude(int encoded_value, int sign_bit_index)
{
    int direction_bit = (encoded_value >> sign_bit_index) & 1;

    int magnitude_mask = (1 << sign_bit_index) - 1;

    int magnitude = encoded_value & magnitude_mask;

    return direction_bit ? -magnitude : magnitude;
}

std::map<int, float> FeetechMotorsBus::_normalize(
    const std::map<int, int>& ids_values)
{
    if (calibration_.empty())
    {
        throw std::runtime_error("No calibration registered.");
    }

    std::map<int, float> normalized_values;

    for (const auto& [id, val] : ids_values)
    {
        std::string motor = _id_to_name(id);

        int min_ = calibration_.at(motor).range_min;
        int max_ = calibration_.at(motor).range_max;

        bool drive_mode = apply_drive_mode && calibration_.at(motor).drive_mode;

        if (max_ == min_)
        {
            throw std::runtime_error(
                "Invalid calibration for motor '" + motor +
                "': min and max are equal.");
        }

        int bounded_val = std::min(max_, std::max(min_, val));

        if (motors_.at(motor).norm_mode == MotorNormMode::RANGE_M100_100)
        {
            float norm =
                ((static_cast<float>(bounded_val - min_) /
                 (max_ - min_)) *
                 200.0f) -
                100.0f;

            normalized_values[id] = drive_mode ? -norm : norm;
        }
        else if (motors_.at(motor).norm_mode == MotorNormMode::RANGE_0_100)
        {
            float norm =
                (static_cast<float>(bounded_val - min_) /
                 (max_ - min_)) *
                100.0f;

            normalized_values[id] = drive_mode ? (100.0f - norm) : norm;
        }
        else if (motors_.at(motor).norm_mode == MotorNormMode::DEGREES)
        {
            float mid = (min_ + max_) / 2.0f;

            int max_res =
                model_resolution_table.at(_id_to_model(id)) - 1;

            normalized_values[id] =
                (val - mid) * 360.0f / max_res;
        }
        else
        {
            throw std::runtime_error("Normalization mode not implemented.");
        }
    }

    return normalized_values;
}

std::string FeetechMotorsBus::_get_motor_model(const NameOrID& motor)
{
    if (std::holds_alternative<std::string>(motor))
    {
        const std::string& name = std::get<std::string>(motor);
        return motors_.at(name).model;
    }
    else if (std::holds_alternative<int>(motor))
    {
        int id = std::get<int>(motor);
        return _id_to_model_dict.at(id);
    }
    else
    {
        throw std::invalid_argument("motor should be int or string");
    }
}

bool FeetechMotorsBus::_has_different_ctrl_tables() const
{
    if (models().size() < 2)
        return false;

    const auto& first_table =
        get_ctrl_table(model_ctrl_table, models().front());

    for (size_t i = 1; i < models().size(); ++i)
    {
        const auto& table =
            get_ctrl_table(model_ctrl_table, models()[i]);

        if (first_table != table)
            return true;
    }

    return false;
}

void FeetechMotorsBus::_setup_sync_reader(
    const std::vector<int>& motor_ids,
    int addr,
    int length)
{
    sync_reader.clearParam();

    sync_reader.start_address_ = addr;
    sync_reader.data_length_ = length;

    for (int id : motor_ids)
    {
        sync_reader.addParam(id);
    }
}

std::pair<std::unordered_map<std::string, Value>, std::unordered_map<std::string, Value>> FeetechMotorsBus::record_ranges_of_motion(
    const std::optional<std::variant<NameOrID,std::vector<NameOrID>>>& motors,
    bool display_values)
{
    std::vector<NameOrID> motors_list;

    if (!motors.has_value())
    {
        for (const auto& [name,_] : motors_)
            motors_list.push_back(name);
    }
    else if (std::holds_alternative<NameOrID>(*motors))
    {
        motors_list.push_back(std::get<NameOrID>(*motors));
    }
    else if (std::holds_alternative<std::vector<NameOrID>>(*motors))
    {
        motors_list = std::get<std::vector<NameOrID>>(*motors);
    }
    else
        throw std::invalid_argument("Invalid motors argument");

    std::vector<std::string> motor_names;
    motor_names.reserve(motors_list.size());

    for (const auto& m : motors_list)
    {
        if (std::holds_alternative<std::string>(m))
            motor_names.push_back(std::get<std::string>(m));
        else
            motor_names.push_back(_id_to_name(std::get<int>(m)));
    }

    auto start_positions = sync_read("Present_Position", motor_names, false);

    auto mins  = start_positions;
    auto maxes = start_positions;

    // bool user_pressed_enter = false;
    std::atomic<bool> user_pressed_enter(false);

    std::thread input_thread([&](){
    std::cin.get();
    user_pressed_enter = true;
    });

    while (!user_pressed_enter)
    {
        auto positions = sync_read("Present_Position", motor_names, false);

        for (auto& [motor, min_val] : mins)
        {
            int pos = std::get<int>(positions[motor]);
            int current_min = std::get<int>(min_val);
            min_val = std::min(pos,current_min);
        }

        for (auto& [motor, max_val] : maxes)
        {
            int pos = std::get<int>(positions[motor]);
            int current_max = std::get<int>(max_val);
            max_val = std::max(pos,current_max);
        }

        if (display_values)
        {
            std::cout << "\n-------------------------------------------\n";
            std::cout << std::left << std::setw(15) << "NAME"
                      << " | " << std::setw(6) << "MIN"
                      << " | " << std::setw(6) << "POS"
                      << " | " << std::setw(6) << "MAX" << "\n";

            for (const auto& motor : motor_names)
            {
                std::cout << std::left << std::setw(15) << motor
                          << " | " << std::setw(6) << std::get<int>(mins[motor])
                          << " | " << std::setw(6) << std::get<int>(positions[motor])
                          << " | " << std::setw(6) << std::get<int>(maxes[motor])
                          << "\n";
            }
        }

        // if (std::cin.rdbuf()->in_avail() > 0)
        // {
        //     std::cin.get();
        //     user_pressed_enter = true;
        // }

        if (display_values && !user_pressed_enter)
        {
            std::cout << "\033[" << motor_names.size() + 3 << "A";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    input_thread.join();

    std::vector<std::string> same_min_max;

    for (const auto& motor : motor_names)
    {
        if (std::get<int>(mins[motor]) == std::get<int>(maxes[motor]))
            same_min_max.push_back(motor);
    }

    if (!same_min_max.empty())
        throw std::runtime_error("Some motors have identical min and max values");

    return {mins,maxes};
}

void FeetechMotorsBus::write_calibration(
    const std::unordered_map<std::string, MotorCalibration>& calibration_dict)
{
    for (const auto& [motor, calibration] : calibration_dict)
    {
        if (protocol_version_ == 0)
        {
            write("Homing_Offset", motor, calibration.homing_offset);
        }

        write("Min_Position_Limit", motor, calibration.range_min);
        write("Max_Position_Limit", motor, calibration.range_max);
    }

    calibration_ = calibration_dict;
}

void FeetechMotorsBus::disconnect(bool disable_torque_flag)
{
    if (!is_connected())
    {
        throw DeviceNotConnectedError(
            "FeetechMotorsBus('" + port_ + "') is not connected. "
            "Try running FeetechMotorsBus.connect() first."
        );
    }

    if (disable_torque_flag)
    {
        port_handler.clearPort();
        port_handler.is_using = false;

        disable_torque({}, 5);
    }

    port_handler.closePort();

    std::cout << "[DEBUG] FeetechMotorsBus disconnected." << std::endl;
}

void FeetechMotorsBus::sync_write(
    const std::string & data_name,
    const std::variant<std::monostate, Value, std::map<std::string, Value>> & values,
    bool normalize,
    int num_retry
)
{
    if (!is_connected()) {
        throw DeviceNotConnectedError(
            "FeetechMotorsBus('" + port_ + "') is not connected. "
            "You need to run FeetechMotorsBus.connect()."
        );
    };

    std::map<int, Value> ids_values = _get_ids_values_dict(values);

    std::vector<std::string> models;
    models.reserve(ids_values.size());
    for (const auto & [id_, _] : ids_values) {
        models.push_back(_id_to_model(id_));
    }

    if (_has_different_ctrl_tables()) {
        assert_same_address(model_ctrl_table, models, data_name);
    }

    if (models.empty()) {
        throw std::invalid_argument(
            "sync_write('" + data_name + "') received no motor models."
        );
    }

    const std::string & model = models.front();
    auto [addr, length] = get_address(model_ctrl_table, model, data_name);

    std::map<int, int> raw_ids_values;

    if (normalize && normalized_data.count(data_name) > 0) {
        raw_ids_values = _unnormalize(_values_to_double(ids_values));
    } else {
        raw_ids_values = _values_to_raw_int(ids_values);
    }

    raw_ids_values = _encode_sign(data_name, raw_ids_values);

    std::ostringstream oss;
    oss << "Failed to sync write '" << data_name << "' with ids_values={";

    bool first = true;
    for (const auto & [id_, value] : ids_values) {
        if (!first) {
            oss << ", ";
        }

        oss << id_ << ": ";
        std::visit([&oss](const auto & v) {
            oss << v;
        }, value);

        first = false;
    }

    oss << "} after " << (num_retry + 1) << " tries.";

    const std::string err_msg = oss.str();

    _sync_write(
    addr,
    length,
    raw_ids_values,
    num_retry,
    true,
    err_msg
    );
}

std::map<int, Value> FeetechMotorsBus::_get_ids_values_dict(
    const std::variant<std::monostate, Value, std::map<std::string, Value>> & values)
{
    std::map<int, Value> ids_values;
    // std::map<int, int> ids_values;

    if (std::holds_alternative<Value>(values)) {
        const Value & single_value = std::get<Value>(values);

        // int raw_value = std::visit([](auto v) { return static_cast<int>(v);}, single_value);

        for (int id : this->ids()) {
            ids_values[id] = single_value;
            // ids_values[id] = raw_value;
        }
        return ids_values;
    }
    else if (std::holds_alternative<std::map<std::string, Value>>(values)) {
        const std::map<std::string, Value> & named_values = std::get<std::map<std::string, Value>>(values);

        for (const auto & [motor_name, val] : named_values) {
                auto it = motors_.find(motor_name);
                if (it == motors_.end()) {
                    throw std::out_of_range("Unknown motor: " + motor_name);
                }

                int id = it->second.id;

                ids_values[id] = val;
        }
        return ids_values;
    }
    else {
        throw std::invalid_argument("'values' is expected to be a single value or a dict.");
    }
}

int FeetechMotorsBus::_sync_write(
    int addr,
    int length,
    const std::map<int, int> & ids_values,
    int num_retry,
    bool raise_on_error,
    const std::string & err_msg
)
{
    _setup_sync_writer(ids_values, addr, length);

    int comm = 0;

    for (int n_try = 0; n_try < 1 + num_retry; ++n_try) {
        comm = sync_writer.txPacket();

        if (_is_comm_success(comm)) {
            break;
        }

        std::ostringstream oss;
        oss << "Failed to sync write @addr=" << addr
            << " (length=" << length << ") with ids_values={";

        bool first = true;
        for (const auto & [id_, value] : ids_values) {
            if (!first) {
                oss << ", ";
            }
            oss << id_ << ": " << value;
            first = false;
        }

        oss << "} (n_try=" << n_try << "): "
            << packet_handler.getTxRxResult(comm);

        std::cerr << oss.str() << std::endl;
    }

    if (!_is_comm_success(comm) && raise_on_error) {
        throw ConnectionError(
            err_msg + " " + packet_handler.getTxRxResult(comm)
        );
    }

    return comm;
}

void FeetechMotorsBus::_setup_sync_writer(
    const std::map<int, int> & ids_values,
    int addr,
    int length
)
{
    sync_writer.clearParam();
    sync_writer.start_address_ = addr;
    sync_writer.data_length_ = length;
    
    for (const auto & [id_, value] : ids_values) {
        std::vector<uint8_t> data = _serialize_data(value, length);
        sync_writer.addParam(id_, data);
    }
}

std::map<int, double> FeetechMotorsBus::_values_to_double(
    const std::map<int, Value> & ids_values
)
{
    std::map<int, double> out;

    for (const auto & [id_, value] : ids_values) {
        out[id_] = std::visit([](const auto & v) -> double {
            return static_cast<double>(v);
        }, value);
    }

    return out;
}

std::map<int, int> FeetechMotorsBus::_values_to_raw_int(
    const std::map<int, Value> & ids_values
)
{
    std::map<int, int> raw_ids_values;

    for (const auto & [id_, value] : ids_values) {
        raw_ids_values[id_] = std::visit([](const auto & v) -> int {
            return static_cast<int>(v);
        }, value);
    }

    return raw_ids_values;
}

// std::unordered_map<std::string, MotorCalibration> FeetechMotorsBus::setCalibration(std::optional<std::unordered_map<std::string, MotorCalibration>> calib) {
//     if (calib && !calib->empty()) {
//         calibration_ = std::move(*calib);
//     } else {
//         calibration_ = std::unordered_map<std::string, MotorCalibration>{};
//     }
//     return calibration_;
// }

// void FeetechMotorsBus::_handshake(){
//     _assert_motors_exist();
//     _assert_same_firmware();
// }

// bool FeetechMotorsBus::_is_comm_success(int comm) const
// {
//     return comm == _comm_success;
// }

// bool FeetechMotorsBus::_is_error(int error) const
// {
//     return error != _no_error;
// }

// std::tuple<int, int> FeetechMotorsBus::get_address(feetech::ModelControlTable model_ctrl_table, std::string model, const std::string& data_name) {
//     return {1, 2};
// }

// std::unordered_map<int, int> FeetechMotorsBus::_unnormalize(const std::unordered_map<int, double>& ids_values) {
//     if (calibration_.empty()){
//         throw std::runtime_error("No calibration registered.");
//     }

//     std::unordered_map<int,int> unnormalized_values;

//     for (const auto& [id_, val_input] : ids_values) {
//         double val = val_input;

//         std::string motor = _id_to_name(id_);

//         int min_ = calibration_.at(motor).range_min;
//         int max_ = calibration_.at(motor).range_max;

//         bool drive_mode = apply_drive_mode && calibration_.at(motor).drive_mode;

//     //     if (max_ == min_) {
//     //         throw std::invalid_argument(
//     //             "Invalid calibration for motor '" + motor + "': min and max are equal."
//     //         );
//     //     }

//     //     auto norm_mode = motors_.at(motor).norm_mode;

//     //     if (norm_mode == MotorNormMode::RANGE_M100_100) {
//     //         val = drive_mode ? -val : val;

//     //         double bounded_val = std::min(100.0, std::max(-100.0, val));

//     //         unormalized_values[id_] = static_cast<int>(((bounded_val + 100) / 200) * (max_ - min_) + min_);
//     //     }
//     //     else if (norm_mode == MotorNormMode::RANGE_0_100) {

//     //         val = drive_mode ? (100 - val) : val;
//     //         double bounded_val = std::min(100.0, std::max(0.0, val));
//     //         unnormalized_values[id_] = static_cast<int>((bounded_val / 100) * (max_ - min_) + min_);
//     //     }
//     //     else if (norm_mode == MotorNormMode::DEGREES) {
//     //         double mid = (min_ + max_) / 2.0;

//     //         int max_res = model_resolution_table_.at(_id_to_model(id_)) - 1;

//     //         unnormalized_values[id_] = static_cast<int>((val * max_res / 360.0) + mid);
//     //     }
//     //     else {
//     //         throw std::logic_error("MotorNormMode not implemented.");
//     //     }
//     }

//     return unnormalized_values;
// }

// int FeetechMotorsBus::encode_sign_magnitude(int value, int sign_bit_index)
// {
//     int max_magnitude = (1 << sign_bit_index) - 1;
//     int magnitude = std::abs(value);

//     if (magnitude > max_magnitude) {
//         throw std::invalid_argument(
//             "Magnitude " + std::to_string(magnitude) +
//             " exceeds " + std::to_string(max_magnitude) +
//             " (max for sign_bit_index=" + std::to_string(sign_bit_index) + ")"
//         );
//     }

//     int direction_bit = (value < 0) ? 1 : 0;

//     return (direction_bit << sign_bit_index) | magnitude;
// }

// std::unordered_map<int, int> FeetechMotorsBus::_encode_sign(const std::string& data_name, std::unordered_map<int, int> ids_values) {
//     for (auto& [id_, val] : ids_values) {
//         std::string model = _id_to_model(id_);

//         auto table_it = model_encoding_table.find(model);

//         if (table_it != model_encoding_table.end()) {

//             const auto& encoding_table = table_it->second;

//             auto sign_it = encoding_table->find(data_name);

//             if (sign_it != encoding_table->end()) {

//                 int sign_bit = sign_it->second;

//                 val = encode_sign_magnitude(val, sign_bit);
//             }
//         }
//     }

//     return ids_values;
// }

// void FeetechMotorsBus::write(const std::string& data_name, const std::string& motor, Value value, bool normalize, int num_retry) {
//     if (!is_connected()) {
//         throw DeviceNotConnectedError();
//     };

//     int id_ = motors_[motor].id;
    
//     std::string model = motors_[motor].model;

//     auto [addr, length] = get_address(model_ctrl_table_, model, data_name);

//     if (normalize && normalized_data.count(data_name)) {
//         value = _unnormalize({{id_, std::get<float>(value)}})[id_];
//     }

//     value = _encode_sign(data_name, {{id_, std::get<int>(value)}})[id_];

//     std::string err_msg =
//         "Failed to write '" + data_name +
//         "' on id=" + std::to_string(id_) +
//         " with '" + std::to_string(std::get<int>(value)) +
//         "' after " + std::to_string(num_retry + 1) + " tries.";

//     _write(addr, length, id_, value, num_retry, true, err_msg);
// }

// std::vector<uint8_t> split_into_byte_chunks(uint32_t value, int length)
// {
//     std::vector<uint8_t> data;

//     if (length == 1) {
//         data = { static_cast<uint8_t>(value) };
//     }
//     else if (length == 2) {
//         data = {
//             static_cast<uint8_t>(SCS_LOBYTE(value)),
//             static_cast<uint8_t>(SCS_HIBYTE(value))
//         };
//     }
//     else if (length == 4) {
//         data = {
//             static_cast<uint8_t>(SCS_LOBYTE(SCS_LOWORD(value))),
//             static_cast<uint8_t>(SCS_HIBYTE(SCS_LOWORD(value))),
//             static_cast<uint8_t>(SCS_LOBYTE(SCS_HIWORD(value))),
//             static_cast<uint8_t>(SCS_HIBYTE(SCS_HIWORD(value)))
//         };
//     }
//     else {
//         throw std::runtime_error("Unsupported byte length");
//     }

//     return data;
// }

// std::vector<uint8_t> FeetechMotorsBus::_serialize_data(int value, int length) {
//     if (value < 0) {
//         throw std::invalid_argument("Negative values are not allowed: " + std::to_string(value));
//     }

//     uint32_t max_value;

//     switch (length) {
//         case 1:
//             max_value = 0xFF;
//             breal;
//         case 2:
//             max_value = 0xFFFF;
//         case 4:
//             max_value = 0xFFFFFFFF;
//             break;
//         default:
//             throw std::runtime_error("Unsupported byte size: " + std::to_string(length) + ". Expeced [1, 2, 4].");
//     }
    

//     if (static_cast<uint32_t>(value) > max_value) {
//         throw std::invalid_argument(
//             "Value " + std::to_string(value) +
//             " exceeds the maximum for " + std::to_string(length) +
//             " bytes (" + std::to_string(max_value) + ")"
//         );
//     }

//     return _split_into_byte_chunks(value, length);
// }

// std::tuple<int,int> FeetechMotorsBus::_write(
//     int addr,
//     int length,
//     int motor_id,
//     int value,
//     int num_retry,
//     bool raise_on_error,
//     const std::string& err_msg)
//     {
//         auto data = _serialize_data(value, length);

//         int comm = 0;
//         int error = 0;

//         for (int n_try = 0; n_try < 1 + num_retry; ++n_try) {
//             std::tie(comm, error) =  packet_handler.writeTxRx(port_handler, motor_id, addr, length, data);

//             if (_is_comm_success(comm)) {
//                 break;
//             }

//             std::cout << "Failed to sync write @addr=" << addr
//                       << " (length=" << length << ") on id=" << motor_id
//                       << " with value=" << value
//                       << " (try=" << n_try << "): "
//                       << packet_handler.getTxRxResult(comm)
//                       << std::endl; 
//             }

//     if (!_is_comm_success(comm) && raise_on_error) {

//         throw std::runtime_error(
//             err_msg + " " + packet_handler.getTxRxResult(comm)
//         );
//     }
//     else if (_is_error(error) && raise_on_error) {

//         throw std::runtime_error(
//             err_msg + " " + packet_handler.getRxPacketError(error)
//         );
//     }

//     return {comm, error};
// }































































// std::optional<std::unordered_map<int, int>> FeetechMotorsBus::broadcast_ping(const int num_retry, bool raise_on_error)
//     {
//         _assert_protocol_is_compatible("broadcast_ping");

//         std::unordered_map<int, int> ids_status;
//         int comm = -1;

//         for (int n_try = 0; n_try < (1 + num_retry); ++n_try) 
//         {
//             std::pair<std::unordered_map<int,int>, int> result = _broadcast_ping();
//             ids_status = std::move(result.first);
//             comm = result.second;

//             if (_is_comm_success(comm)) {
//                 break;
//             }

//             RCLCPP_DEBUG(logger_,"Broadcast ping failed on port '%s' (n_try=%d)",port_.c_str(), n_try);
//             RCLCPP_DEBUG(logger_, "%s", packet_handler_.getTxRxResult(comm).c_str());
//         }

//         if (!_is_comm_success(comm))
//             {
//                 if (raise_on_error)
//                 {
//                     throw ConnectionError(packet_handler_.getTxRxResult(comm));                         
//                 }
//                 return std::nullopt;
//             }

//         std::unordered_map<int, int> ids_errors;
//         for (const auto& kv : ids_status) {
//             const int id_ = kv.first;
//             const int status = kv.second;
//             if (_is_error(status))
//             {
//                 ids_errors.emplace(id_, status);
//             }
//         }

//         if (!ids_errors.empty()) {
//             std::string msg = "Some motors found retourned an error status:\n";

//             for (const auto& kv : ids_errors) {
//                 msg += "  id " + std::to_string(kv.first) + ": "
//                 + packet_handler_.getRxPacketError(kv.second) + "\n";
//             }
//             RCLCPP_ERROR(logger_, "%s", msg.c_str());
//         }

//         std::vector<int> ids;
//         ids.reserve(ids_status.size());
//         for (const auto& kv : ids_status) {
//             ids.push_back(kv.first);
//         }

//         return _read_model_number(ids, raise_on_error);
//     }

// std::pair<std::unordered_map<int,int>, int> FeetechMotorsBus::_broadcast_ping()
// {
//     std::unordered_map<int,int> data_list;

//     constexpr std::size_t status_length = 6;
//     std::size_t rx_length = 0;
//     const std::size_t wait_length = status_length * static_cast<std::size_t>(scservo_def_table::MAX_ID);

//     std::vector<std::uint8_t> txpacket(6,0);

//     const double tx_time_per_byte = (1000.0 / static_cast<double>(port_handler_.getBaudRate())) * 10.0;

//     txpacket[protocol_packet_handler_params::PKT_ID] = scservo_def_table::BROADCAST_ID;
//     txpacket[protocol_packet_handler_params::PKT_LENGTH] = 2;
//     txpacket[protocol_packet_handler_params::PKT_INSTRUCTION] = scservo_def_table::INST_PING;

//     int result = packet_handler_.txPacket(port_handler_, tx_packet);
//     if (result != scservo_def_table::COMM_SUCCESS) {
//         port_handler_.is_using = false;
//         return {data_list, result};
//     }

//     port_handler_.setPacketTimeoutMillis(
//         (static_cast<double>(wait_length) * tx_time_per_byte)
//         + (3.0 * static_cast<double>(scservo_def_table::MAX_ID))
//         + 16.0
//     );

//     std::vector<std::uint8_t> rxpacket;
//     rxpacket.reserve(wait_length);

//     while (!port_handler_.isPacketTimeout() && rx_length < wait_length) {
//         const std::size_t to_read = wait_length - rx_length;
//         std::vector<std::uint8_t> chunk = port_handler_.readPort(to_read);

//         if (!chunk.empty()) {
//             rxpacket.insert(rxpacket.end(), chunk.begin(), chunk.end());
//             rx_length = rxpacket.size();
//         }
//         else
//         {
//             rx_length = rxpacket.size();
//         }


//     }

//     port_handler_.is_using = false;

//     if (rx_length == 0) 
//     {
//         return {data_list, scservo_def_table::COMM_RX_TIMEOUT};
//     }

//     while (true) 
//     {
//         if (rx_length < status_length) 
//         {
//             return {data_list, scservo_def_table::COMM_RX_CORRUPT};
//         }
    
//         std::size_t idx = 0;
//         bool found = false;
//         for (std::size_t i = 0; i+1 < rx_length; ++i) 
//         {
//             if (rx_packet[i] == 0xFF && rxpacket[i+1] = 0xFF)
//             {
//                 idx = i;
//                 found = true;
//                 break;
//             }
//         }

//         if (!found) 
//         {
//             if (rx_length > 1)
//             {
//                 rxpacket.erase(rxpacket.begin(), rxpacket.end() - 1);
//                 rx_length = rxpacket.size();
//                 continue;
//             }
//             return {data_list, scservo_def_table::COMM_RX_CORRUPT};
//         }

//         if (idx == 0) 
//         {
//             std::uint32_t checksum = 0;
//             for (std::size_t j = 2; j < status_length - 1; ++j) 
//             {
//                 checksum += rxpacket[j];
//             }

//             checksum = (~checksum) & 0xFF;

//             if (rxpacket[status_length - 1] == static_cast<std::uint8_t>(checksum))
//             {
//                 result = scservo_def_table::COMM_SUCCESS;

//                 const int id = static_cast<int>(rxpacket[protocol_packet_handler_params::PKT_ID]);
//                 const int error = static_cast<int>(rxpacket[protocol_packet_handler_params::PKT_ERROR]);

//                 data_list[id] = error;

//                 rxpacket.erase(rxpacket.begin(), rxpacket.begin() + static_cast<std::ptrdiff_t>(status_length));
//                 rx_length -= status_length;

//                 if (rx_length == 0) 
//                 {
//                     return {data_list, result};
//                 }
//             }
//             else
//             {
//                 result = scservo_def_table::COMM_RX_CORRUPT;

//                 rxpacket.erase(rxpacket.begin(), rxpacket.begin() + 2);
//                 rx_length -=2;
//             }
//         }
//         else
//         {
//             rxpacket.erase(rxpacket.begin(), rxpacket.begin() + static_cast<std::ptrdiff_t>(idx));
//             rx_length -= idx;
//         }
//     }
// }

// bool MotorsBus::_is_comm_success(int comm) {
//     return comm == _comm_success;
// }

// bool _is_error(const int error)
// {
//     return error != _no_error;
// }

// std::unordered_map<int, int> FeetechMotorsBus::_read_model_number(const std::vector<int>& motor_ids, bool raise_on_error) {
//     std::unordered_map<int, int> model_numbers;
//     model_numbers.reserve(motor_ids.size());

//     for (const int id_ : motor_ids) {
//         auto [model_nb, comm, error] =
//         _read(feetech::MODEL_NUMBER.address, feetech::MODEL_NUMBER.size, id_, raise_on_error);

//         if (!_is_comm_success(comm) || _is_error(error)) {
//             continue;
//         }

//         model_numbers[id_] = model_nb;
//     }

//     return model_numbers;
// }

