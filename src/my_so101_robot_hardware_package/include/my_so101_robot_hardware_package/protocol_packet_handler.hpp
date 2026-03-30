#pragma once

#include "my_so101_robot_hardware_package/scservo_def.hpp"
#include "port_handler.hpp"
#include <tuple>
#include <vector>

constexpr int TXPACKET_MAX_LEN = 250;
constexpr int RXPACKET_MAX_LEN = 250;

// for Protocol Packet
constexpr int PKT_HEADER0 = 0;
constexpr int PKT_HEADER1 = 1;
constexpr int PKT_ID = 2;
constexpr int PKT_LENGTH = 3;
constexpr int PKT_INSTRUCTION = 4;
constexpr int PKT_ERROR = 4;
constexpr int PKT_PARAMETER0 = 5;

// for Protocol Error bit 
constexpr int ERRBIT_VOLTAGE = 1;
constexpr int ERRBIT_ANGLE = 2;
constexpr int ERRBIT_OVERHEAT = 4;
constexpr int ERRBIT_OVERELE = 8;
constexpr int ERRBIT_OVERLOAD = 32;

// class protocol_packet_handler() {
//     explicit protocol_packet_handler(PortHandler& portHandler, int protocol_end);
//     PortHandler& portHandler_;
//     int scs_end_;
// };

class protocol_packet_handler 
{
public:
    protocol_packet_handler() = default;
    std::tuple<int, int, int> ping(PortHandler& port, int scs_id);
    std::tuple<std::vector<uint8_t>, int, int> txRxPacket(PortHandler& port, std::vector<uint8_t>& txpacket);
    std::tuple<std::vector<uint8_t>, int, int> readTxRx(PortHandler& port, int scs_id, int address, int length);
    int txPacket(PortHandler& port, std::vector<uint8_t>& txpacket);
    std::tuple<std::vector<uint8_t>, int> rxPacket(PortHandler& port);
    std::tuple<uint32_t, int, uint8_t> read1ByteTxRx(PortHandler* port, uint8_t scs_id, uint16_t address);
    std::tuple<uint32_t, int, uint8_t> read2ByteTxRx(PortHandler* port, uint8_t scs_id, uint16_t address);
    std::tuple<uint32_t, int, uint8_t> read4ByteTxRx(PortHandler* port, uint8_t scs_id, uint16_t address);
    std::tuple<std::vector<uint8_t>, int, uint8_t> readTxRx(PortHandler* port, uint8_t scs_id, uint8_t address, uint8_t length);
    std::tuple<std::vector<uint8_t>, int, uint8_t> txRxPacket(PortHandler* port, std::vector<uint8_t> txpacket);
    std::string getTxRxResult(int result);
    std::string getRxPacketError(uint8_t error);
    std::pair<int,int> writeTxRx(PortHandler* port, uint8_t scs_id, uint8_t address, uint8_t length, const std::vector<uint8_t>& data);
    int syncReadTx(PortHandler& port, int start_address, int data_length, const std::vector<uint8_t>& param, int param_length);    
    std::tuple<std::vector<uint8_t>, int, uint8_t> readRx(PortHandler* port, uint8_t scs_id, uint8_t length);
    int syncWriteTxOnly(PortHandler * port, int start_address, int data_length, const std::vector<uint8_t> & param, int param_length);
};