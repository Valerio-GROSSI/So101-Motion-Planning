#pragma once

#include "my_so101_robot_hardware_package/scservo_def.hpp"

#include <vector>
#include <unordered_map>
#include <cstdint>

class PortHandler;
class protocol_packet_handler;

class GroupSyncRead
{
public:
    GroupSyncRead(PortHandler* port, protocol_packet_handler* ph, int start_address, int data_length);
    void clearParam();

    PortHandler* port_;
    protocol_packet_handler* ph_;

    int start_address_;
    int data_length_;

    bool last_result;
    bool is_param_changed;

    std::vector<uint8_t> param_;
    std::unordered_map<int, std::vector<uint8_t>> data_dict_;
    int txRxPacket();
    int txPacket();
    int rxPacket();
    void makeParam();
    bool addParam(int scs_id);
    int getData(int scs_id, int address, int data_length);
    bool isAvailable(int scs_id, int address, int data_length);
};