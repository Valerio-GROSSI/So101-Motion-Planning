#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "my_so101_robot_hardware_package/scservo_def.hpp"

class PortHandler;
class protocol_packet_handler;

class GroupSyncWrite
{
public:
    GroupSyncWrite(
        PortHandler * port,
        protocol_packet_handler * ph,
        int start_address,
        int data_length
    );

    void makeParam();

    bool addParam(uint8_t scs_id, const std::vector<uint8_t> & data);
    void removeParam(uint8_t scs_id);
    bool changeParam(uint8_t scs_id, const std::vector<uint8_t> & data);
    void clearParam();

    int txPacket();

    PortHandler * port_;
    protocol_packet_handler * ph_;
    int start_address_;
    int data_length_;

    bool is_param_changed;
    std::vector<uint8_t> param;
    std::unordered_map<uint8_t, std::vector<uint8_t>> data_dict;
};