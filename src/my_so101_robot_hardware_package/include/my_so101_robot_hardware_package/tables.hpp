#pragma once

#include <cstdint>
#include <unordered_map>
#include <string>
#include <vector>

namespace tables {

// struct Reg {
//     std::uint16_t address{};
//     std::uint8_t size{};
// };

// struct Reg {
//     int address;
//     int size;
// };

struct Reg
{
    int address;
    int size;
    bool readonly;

    bool operator==(const Reg& other) const
    {
        return address == other.address &&
               size == other.size &&
               readonly == other.readonly;
    }

    bool operator!=(const Reg& other) const
    {
        return !(*this == other);
    }
};

using ControlTable = std::unordered_map<std::string, Reg>;
using BaudrateTable = std::unordered_map<int, int>;
using EncodingTable = std::unordered_map<std::string, int>;
using StringIntMap = std::unordered_map<std::string, int>;
using ModelControlTable = std::unordered_map<std::string, const ControlTable*>;
using ModelBaudrateTable = std::unordered_map<std::string, const BaudrateTable*>;
using ModelEncodingTable = std::unordered_map<std::string, const EncodingTable*>;

extern const Reg FIRMWARE_MAJOR_VERSION;
extern const Reg FIRMWARE_MINOR_VERSION;
extern const Reg MODEL_NUMBER;
extern const ControlTable STS_SMS_SERIES_CONTROL_TABLE;
extern const ControlTable SCS_SERIES_CONTROL_TABLE;
extern const BaudrateTable STS_SMS_SERIES_BAUDRATE_TABLE;
extern const BaudrateTable SCS_SERIES_BAUDRATE_TABLE;
extern const EncodingTable EMPTY_ENCODING_TABLE;
extern const EncodingTable STS_SMS_SERIES_ENCODINGS_TABLE;
extern const ModelControlTable MODEL_CONTROL_TABLE;
extern const ModelBaudrateTable MODEL_BAUDRATE_TABLE;
extern const ModelEncodingTable MODEL_ENCODING_TABLE;
extern const StringIntMap MODEL_RESOLUTION;
extern const StringIntMap MODEL_NUMBER_TABLE;
extern const StringIntMap MODEL_PROTOCOL;
extern const std::vector<int> SCAN_BAUDRATES;
} // namespace tables