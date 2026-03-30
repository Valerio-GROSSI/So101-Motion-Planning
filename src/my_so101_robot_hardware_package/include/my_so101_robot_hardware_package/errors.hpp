#pragma once

#include <stdexcept>
#include <string>

// ConnectionError
class ConnectionError : public std::runtime_error {
public:
    explicit ConnectionError(const std::string& message) : std::runtime_error(message) {}
};

// DeviceNotConnectedError
class DeviceNotConnectedError : public ConnectionError {
public:
    explicit DeviceNotConnectedError(
        const std::string& message = 
        "This device is not connected. Try calling 'connect()' first."
    )
        : ConnectionError(message) {}
};

// DeviceAlreadyConnectedError
class DeviceAlreadyConnectedError : public ConnectionError {
public:
    explicit DeviceAlreadyConnectedError(
        const std::string& message = "This device is already connected. Try not calling 'connect()' twice."
    )
        : ConnectionError(message) {}
};