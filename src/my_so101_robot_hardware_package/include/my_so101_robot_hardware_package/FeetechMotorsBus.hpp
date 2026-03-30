#pragma once

#include <iostream>
#include <variant>
#include <unordered_set>
#include <string>
#include <vector>
#include <cstdint>
#include "my_so101_robot_hardware_package/tables.hpp"
#include "my_so101_robot_hardware_package/motors_bus.hpp"
#include <unordered_map>
#include <optional>
#include <memory>
#include "my_so101_robot_hardware_package/scservo_sdk.hpp"
#include "my_so101_robot_hardware_package/scservo_def.hpp"
#include <stdexcept>
#include "my_so101_robot_hardware_package/errors.hpp"
#include <exception>
#include <sstream>
#include <tuple>
#include <set>
#include <map>
#include <iomanip>
#include <thread>
#include <chrono>
#include <atomic>
// #include <utility>


using NameOrID = std::variant<std::string, int>;
using Value = std::variant<int, float>;

constexpr int DEFAULT_PROTOCOL_VERSION = 0;
constexpr int DEFAULT_BAUDRATE = 1000000; //115200
constexpr int DEFAULT_TIMEOUT_MS = 1000;

inline std::unordered_set<std::string> NORMALIZED_DATA = {"Goal_Position", "Present_Position"};

enum class OperatingMode {
    POSITION = 0,
    VELOCITY = 1,
    PWM = 2,
    STEP = 3
};

enum class DriveMode {
    NON_INVERTED = 0,
    INVERTED = 1
};

enum class TorqueMode {
    ENABLED = 1,
    DISABLED = 0
};

std::vector<uint8_t> _split_into_byte_chunks(int value, int length);

void patch_setPacketTimeout(int packet_length);

class FeetechMotorsBus : public MotorsBus {
public:
    static const bool apply_drive_mode;
    static const std::vector<int> available_baudrates;
    static const int default_baudrate;
    static const int default_timeout;
    static const tables::ModelBaudrateTable model_baudrate_table;
    static const tables::ModelControlTable& model_ctrl_table;
    static const tables::ModelEncodingTable& model_encoding_table;
    static const tables::StringIntMap& model_number_table;
    static const tables::StringIntMap& model_resolution_table;
    static const std::unordered_set<std::string> normalized_data;

FeetechMotorsBus(
    const std::string& port,
    const std::unordered_map<std::string, Motor>& motors,
    const std::optional<std::unordered_map<std::string, MotorCalibration>> calibration = std::nullopt,
    int protocol_version = DEFAULT_PROTOCOL_VERSION
    );

int protocol_version_;
void _assert_same_protocol();
PortHandler port_handler;
// # HACK: monkeypatch
// self.port_handler.setPacketTimeout = patch_setPacketTimeout.__get__(
//     self.port_handler, scs.PortHandler
// )
protocol_packet_handler packet_handler;
GroupSyncRead sync_reader;
GroupSyncWrite sync_writer;
const int _comm_success = scservo_def::COMM_SUCCESS;
const int _no_error = 0x00;
void connect(bool handshake = true);
bool is_connected() const;
void _connect(bool handshake = true);
void _handshake();
void _assert_motors_exist();
void _assert_same_firmware();
std::vector<int> ids() const;
std::string _id_to_name(int motor_id);
// std::unordered_map<int, std::string> _id_to_name_dict;
std::optional<int> ping(NameOrID motor, int num_retry = 0, bool raise_on_error = false);
int _get_motor_id(const NameOrID& motor);
bool _is_comm_success(int comm) const;
bool _is_error(int error) const;
void set_timeout(std::optional<int> timeout_ms = std::nullopt);
std::unordered_map<int, std::string> _read_firmware_version(const std::vector<int>& motor_ids, bool raise_on_error = false);
std::tuple<int,int,int> _read(int address, int length, int motor_id, int num_retry = 0, bool raise_on_error = true, const std::string& err_msg = "");
void disable_torque(const std::vector<std::string>& motors = {}, int num_retry = 0);
std::vector<std::string> _get_motors_list(const std::optional<std::variant<std::string, std::vector<std::string>>>& motors);
void write(const std::string& data_name, const std::string& motor, Value value, bool normalize = true, int num_retry = 0);
std::pair<int,int> get_address(const tables::ModelControlTable& model_ctrl_table, const std::string& model, const std::string& data_name);
const tables::ControlTable& get_ctrl_table(const tables::ModelControlTable& model_ctrl_table, const std::string& model) const;
std::map<int,int> _unnormalize(const std::map<int,double>& ids_values);
std::map<int,int> _encode_sign(const std::string& data_name, std::map<int,int> ids_values);
int encode_sign_magnitude(int value, int sign_bit_index);
std::pair<int,int> _write(int addr, int length, int motor_id, int value, int num_retry = 0, bool raise_on_error = true, const std::string& err_msg = "");
std::vector<uint8_t> _serialize_data(int value, int length);
void configure_motors();
std::unordered_map<NameOrID, Value>set_half_turn_homings(const std::optional<std::variant<NameOrID, std::vector<NameOrID>>>& motors = std::nullopt);
void reset_calibration(const std::optional<std::variant<NameOrID, std::vector<NameOrID>>>& motors);
std::unordered_map<std::string, Value> sync_read(
    const std::string& data_name,
    const std::optional<std::vector<std::string>>& motors = std::nullopt,
    bool normalize = true,
    int num_retry = 0);
std::unordered_map<std::string, Value>_get_half_turn_homings(const std::unordered_map<std::string, Value>& positions);
void _assert_protocol_is_compatible(const std::string& instruction_name);
void assert_same_address(
    const tables::ModelControlTable& model_ctrl_table,
    const std::vector<std::string>& motor_models,
    const std::string& data_name);
std::pair<std::map<int,int>, int> _sync_read(
    int addr,
    int length,
    const std::vector<int>& motor_ids,
    int num_retry,
    bool raise_on_error,
    const std::string& err_msg);
std::map<int,int> _decode_sign(const std::string& data_name, std::map<int,int> ids_values);
int decode_sign_magnitude(int encoded_value, int sign_bit_index);
std::map<int, float> _normalize(const std::map<int, int>& ids_values);
std::string _get_motor_model(const NameOrID& motor);
bool _has_different_ctrl_tables() const;
void _setup_sync_reader(const std::vector<int>& motor_ids, int addr, int length);
std::pair<std::unordered_map<std::string, Value>, std::unordered_map<std::string, Value>> record_ranges_of_motion(
    const std::optional<std::variant<NameOrID,std::vector<NameOrID>>>& motors = std::nullopt,
    bool display_values = true);
void write_calibration(const std::unordered_map<std::string, MotorCalibration>& calibration_dict);
void disconnect(bool disable_torque_flag = true);

std::map<int, Value> _get_ids_values_dict(const std::variant<std::monostate, Value, std::map<std::string, Value>> & values);

void sync_write(
    const std::string & data_name,
    const std::variant<std::monostate, Value, std::map<std::string, Value>> & values,
    bool normalize = true,
    int num_retry = 0
);

int _sync_write(
    int addr,
    int length,
    const std::map<int, int> & ids_values,
    int num_retry = 0,
    bool raise_on_error = true,
    const std::string & err_msg = ""
);

void _setup_sync_writer(
    const std::map<int, int> & ids_values,
    int addr,
    int length
);

std::map<int, double> _values_to_double(
    const std::map<int, Value> & ids_values
);

std::map<int, int> _values_to_raw_int(
    const std::map<int, Value> & ids_values
);

// std::vector<int> ids() const;

// const tables::StringIntMap& model_number_table = tables::MODEL_NUMBER_TABLE;


// std::string _id_to_name(int motor_id) const;
// bool raise_on_error;

// // std::unordered_map<int, std::string> _model_nb_to_model_dict;

// void write(const std::string& data_name, const std::string& motor, Value value, bool normalize = true, int num_retry = 0);

// tables::ModelControlTable model_ctrl_table_;

// std::tuple<int, int> get_address(tables::ModelControlTable model_ctrl_table, std::string model, const std::string& data_name);

// const std::unordered_set<std::string> normalized_data;

// std::unordered_map<std::string, MotorCalibration> calibration_;

// std::unordered_map<std::string, MotorCalibration> setCalibration(std::optional<std::unordered_map<std::string, MotorCalibration>> calib);

// bool apply_drive_mode = true;

// std::unordered_map<int, int> _unnormalize(const std::unordered_map<int, double>& ids_values);

// std::unordered_map<int, int> _encode_sign(const std::string& data_name, std::unordered_map<int, int> ids_values);

// int encode_sign_magnitude(int value, int sign_bit_index);

// tables::ModelEncodingTable model_encoding_table;

// std::tuple<int,int> _write(int addr, int length, int motor_id, int value, int num_retry, bool raise_on_error, const std::string& err_msg);

// std::vector<uint8_t> _serialize_data(int value, int length);




// protected:
    // void _handshake_impl() override {
    //     { _assert_motors_exist(); 
    //       _assert_same_firmware(); }
    // }
    // bool open_port_impl() override { return port_handler_.openPort(); }

    // void set_timeout_impl(std::optional<int> timeout_ms) override
    // {
    //     const int timeout_ms_ = timeout_ms.value_or(default_timeout_);
    //     port_handler_.setPacketTimeout(timeout_ms_);
    // }

//     void assert_same_firmware();
//     void assert_same_protocol();
//     void FeetechMotorsBus::_assert_protocol_is_compatible(const std::string& instruction_name);
//     std::optional<std::unordered_map<int, int>> broadcast_ping(const int& num_retry = 0, bool raise_on_error = false);
//     std::pair<std::unordered_map<int,int>, int> _broadcast_ping();
//     bool _is_comm_success(int comm);
//     std::unordered_map<int, int> _read_model_number(const std::vector<int>& motor_ids, bool raise_on_error);
//     static void patch_setPacketTimeout(scs::PortHandler& port_handler);

// private:
//     int protocol_version_{DEFAULT_PROTOCOL_VERSION};
//     int default_baudrate_{DEFAULT_BAUDRATE};
//     std::int32_t default_timeout_{DEFAULT_TIMEOUT_MS};

//     scs::PacketHandler packet_handler_;
//     scs::GroupSyncRead sync_reader_;
//     scs::GroupSyncWrite sync_writer_;
//     scs::COMM_SUCCESS _comm_success_;
//     std::uint8_t _no_error = 0x00; 

};