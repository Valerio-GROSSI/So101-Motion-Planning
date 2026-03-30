#pragma once

#include <string>
#include <serial/serial.h>
#include <exception>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>
#include <iostream>
// #include <chrono>

namespace port_handler_const 
{
constexpr int LATENCY_TIMER = 16;
constexpr int DEFAULT_BAUDRATE = 1000000;
};

class PortHandler {
public:
    explicit PortHandler(const std::string& port_name);
    bool is_open;
    int baudrate_;
    double packet_start_time;
    double packet_timeout;
    double tx_time_per_byte;
    bool is_using;
    std::string port_name_;
    serial::Serial ser;    
    bool openPort();
    bool setBaudRate(int baudrate);
    int getCFlagBaud(int baudrate);
    bool setupPort(int cflag_baud);
    void closePort();
    void setPacketTimeoutMillis(int msec);
    double getCurrentTime();
    void setPacketTimeout(int packet_length);
    void clearPort();
    size_t writePort(std::vector<uint8_t>& packet);
    std::vector<uint8_t> readPort(int length);
    bool isPacketTimeout();
    double getTimeSinceStart();

    // void setPortName(const std::string& port_name);
    // const std::string& getPortName() const;
    // bool setBaudRate(int baudrate);
    // int getBaudRate() const;
    // std::size_t getBytesAvailable();
    // std::vector<std::uint8_t> readPort(std::size_t length);
    // std::size_t writePort(const std::vector<std::uint8_t>& packet);
    // void setPacketTimeout(std::size_t packet_length);
    // void setPacketTimeoutMillis(double msec);
    // bool isPacketTimeout();
    // double getCurrentTimeMs() const;
    // double getTimeSinceStartMs();
    // double getCurrentTimeMs() const;
    // double getTimeSinceStartMs();
    // bool is_open() const { return is_open; }


};



// protected:
//     bool setupPort (int cflag_baud);
//     int getCFlagBaud(int baudrate) const;

// private:
    // bool is_open_{false};
    // int baudrate_{params_port_handler::DEFAULT_BAUDRATE};
    // double packet_start_time_ms_{0.0};
    // double packet_timeout_ms_{0.0}; //_ms
    // double tx_time_per_byte_ms_{0.0}; //_ms
    // bool is_using_{false};
    // std::string port_name_;
//     boost::asio::io_context io_;
//     boost::asio::serial_port ser_{io_};

// class PacketHandler {
// public :
//     explicit PacketHandler(int protocol_version);
//     writeTxRx
//     getTxRxResult
//     getRxPacketError
// }