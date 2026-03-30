#pragma once

#include <cstdint>

namespace scservo_def {

inline constexpr int BROADCAST_ID = 0xFE;  // 254
inline constexpr int MAX_ID = 0xFC;  // 252
inline int SCS_END = 0;

// Instruction for DXL Protocol
inline constexpr int INST_PING = 1;
inline constexpr int INST_READ = 2;
inline constexpr int INST_WRITE = 3;
inline constexpr int INST_REG_WRITE = 4;
inline constexpr int INST_ACTION = 5;
inline constexpr int INST_SYNC_WRITE = 131;  //0x83
inline constexpr int INST_SYNC_READ = 130;  // 0x82
// inline constexpr int INST_RESET = 10;  // 0x0A
// inline constexpr int INST_OFSCAL = 11;  // 0x0B

// Communication Result
inline constexpr int COMM_SUCCESS = 0;  // tx or rx packet communication success
inline constexpr int COMM_PORT_BUSY = -1;  // Port is busy (in use)
inline constexpr int COMM_TX_FAIL = -2;  // Failed transmit instruction packet
inline constexpr int COMM_RX_FAIL = -3;  // Failed get status packet
inline constexpr int COMM_TX_ERROR = -4;  // Incorrect instruction packet
inline constexpr int COMM_RX_WAITING = -5;  // Now recieving status packet
inline constexpr int COMM_RX_TIMEOUT = -6;  // There is no status packet
inline constexpr int COMM_RX_CORRUPT = -7;  // Incorrect status packet
inline constexpr int COMM_NOT_AVAILABLE = -9;  //

// Macro for Control Table Value
inline int SCS_GETEND()
{
    return SCS_END;
}

inline void SCS_SETEND(int e)
{
    SCS_END = e;
}

inline int SCS_TOHOST(int a, int b)
{
    if (a & (1 << b))
        return -(a & ~(1 << b));
    else
        return a;
}

inline int SCS_TOSCS(int a, int b)
{
    if (a < 0)
        return (-a | (1 << b));
    else
        return a;
}

inline uint16_t SCS_MAKEWORD(uint8_t a, uint8_t b)
{
    if (SCS_END == 0)
        return (a & 0xFF) | ((b & 0xFF) << 8);
    else
        return (b & 0xFF) | ((a & 0xFF) << 8);
}

inline uint32_t SCS_MAKEDWORD(uint16_t a, uint16_t b)
{
    return (a & 0xFFFF) | ((b & 0xFFFF) << 16);
}

inline uint16_t SCS_LOWORD(uint32_t l)
{
    return l & 0xFFFF;
}

inline uint16_t SCS_HIWORD(uint32_t l)
{
    return (l >> 16) & 0xFFFF;
}

inline uint8_t SCS_LOBYTE(uint16_t w)
{
    if (SCS_END == 0)
        return w & 0xFF;
    else
        return (w >> 8) & 0xFF;
}

inline uint8_t SCS_HIBYTE(uint16_t w)
{
    if (SCS_END == 0)
        return (w >> 8) & 0xFF;
    else
        return w & 0xFF;
}

} // namespace scservo_def