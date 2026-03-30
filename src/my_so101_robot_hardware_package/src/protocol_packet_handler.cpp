#include "my_so101_robot_hardware_package/protocol_packet_handler.hpp"

// protocol_packet_handler::protocol_packet_handler(PortHandler& portHandler, int protocol_end)
//     : portHandler_(portHandler),
//       scs_end_(protocol_end)
// {}

std::tuple<int, int, int> protocol_packet_handler::ping(PortHandler& port, int scs_id)
{
    int model_number = 0;
    int error = 0;

    std::vector<uint8_t> txpacket(6, 0);

    if (scs_id >= scservo_def::BROADCAST_ID)
        return {0, scservo_def::COMM_NOT_AVAILABLE, 0};

    txpacket[PKT_ID] = scs_id;
    txpacket[PKT_LENGTH] = 2;
    txpacket[PKT_INSTRUCTION] = scservo_def::INST_PING;

    // std::cout << "Juste avant txRxPacket Function\n";
    auto [rxpacket, result, err] = txRxPacket(port, txpacket);
    error = err;
    // std::cout << "Juste après txRxPacket Function, result=" << result << ", err=" << err << "\n";

    if (result == scservo_def::COMM_SUCCESS)
    {
        auto [data_read, result2, err2] = readTxRx(port, scs_id, 3, 2);
        error = err2;

        if (result2 == scservo_def::COMM_SUCCESS)
            model_number = scservo_def::SCS_MAKEWORD(data_read[0], data_read[1]);

        result = result2;
    }

    return {model_number, result, error};
}

std::tuple<std::vector<uint8_t>, int, int> protocol_packet_handler::txRxPacket(PortHandler& port, std::vector<uint8_t>& txpacket)
{
    std::vector<uint8_t> rxpacket;
    int error = 0;

    // tx packet
    int result = txPacket(port, txpacket);
    if (result != scservo_def::COMM_SUCCESS)
        return {rxpacket, result, error};

    // (ID == Broadcast ID) == no need to wait for status packet or not available
    if (txpacket[PKT_ID] == scservo_def::BROADCAST_ID)
    {
        port.is_using = false;
        return {rxpacket, result, error};
    }

    // set packet timeout
    if (txpacket[PKT_INSTRUCTION] == scservo_def::INST_READ)
        port.setPacketTimeout(txpacket[PKT_PARAMETER0 + 1] + 6);
    else
        port.setPacketTimeout(6); // HEADER0 HEADER1 ID LENGTH ERROR CHECKSUM

    // rx packet
    while (true)
    {
        auto [packet, res] = rxPacket(port);
        rxpacket = packet;
        result = res;

        if (result != scservo_def::COMM_SUCCESS || txpacket[PKT_ID] == rxpacket[PKT_ID])
        {
            break;
        }
    }

    if (result == scservo_def::COMM_SUCCESS && txpacket[PKT_ID] == rxpacket[PKT_ID])
    {
        error = rxpacket[PKT_ERROR];
    }

    return {rxpacket, result, error};
}

std::tuple<std::vector<uint8_t>, int, int> protocol_packet_handler::readTxRx(PortHandler& port, int scs_id, int address, int length)
{
    std::vector<uint8_t> txpacket(8, 0);
    std::vector<uint8_t> data;

    if (scs_id >= scservo_def::BROADCAST_ID)
        return {data, scservo_def::COMM_NOT_AVAILABLE, 0};

    txpacket[PKT_ID] = scs_id;
    txpacket[PKT_LENGTH] = 4;
    txpacket[PKT_INSTRUCTION] = scservo_def::INST_READ;
    txpacket[PKT_PARAMETER0 + 0] = address;
    txpacket[PKT_PARAMETER0 + 1] = length;

    auto [rxpacket, result, error] = txRxPacket(port, txpacket);

    if (result == scservo_def::COMM_SUCCESS)
    {
        error = rxpacket[PKT_ERROR];

        for (int i = 0; i < length; ++i)
            data.push_back(rxpacket[PKT_PARAMETER0 + i]);
    }

    return {data, result, error};
}

int protocol_packet_handler::txPacket(PortHandler& port, std::vector<uint8_t>& txpacket)
{
    int checksum = 0;
    int total_packet_length = txpacket[PKT_LENGTH] + 4; // HEADER0 HEADER1 ID LENGTH

    if (port.is_using)
        return scservo_def::COMM_PORT_BUSY;

    port.is_using = true;

    // check max packet length
    if (total_packet_length > TXPACKET_MAX_LEN)
    {
        port.is_using = false;
        return scservo_def::COMM_TX_ERROR;
    }

    // make packet header
    txpacket[PKT_HEADER0] = 0xFF;
    txpacket[PKT_HEADER1] = 0xFF;

    // compute checksum
    for (int idx = 2; idx < total_packet_length - 1; ++idx)
        checksum += txpacket[idx];

    txpacket[total_packet_length - 1] = (~checksum) & 0xFF;

    // transmit packet
    port.clearPort();

    int written_packet_length = 0;
    written_packet_length = port.writePort(txpacket);

    if (total_packet_length != written_packet_length)
    {
        port.is_using = false;
        return scservo_def::COMM_TX_FAIL;
    }

    return scservo_def::COMM_SUCCESS;
}

std::tuple<std::vector<uint8_t>, int> protocol_packet_handler::rxPacket(PortHandler& port)
{
    std::vector<uint8_t> rxpacket;

    int result = scservo_def::COMM_TX_FAIL;
    int checksum = 0;
    int rx_length = 0;
    int wait_length = 6; // HEADER0 HEADER1 ID LENGTH ERROR CHECKSUM

    while (true)
    {
        // read from port
        auto data = port.readPort(wait_length - rx_length);
        rxpacket.insert(rxpacket.end(), data.begin(), data.end());
        
        rx_length = rxpacket.size();

        if (rx_length >= wait_length)
        {
            int idx = 0;

            // find packet header
            for (idx = 0; idx < rx_length - 1; ++idx)
            {
                if (rxpacket[idx] == 0xFF && rxpacket[idx + 1] == 0xFF)
                    break;
            }

            if (idx == 0)
            {
                if ((rxpacket[PKT_ID] > 0xFD) ||
                    (rxpacket[PKT_LENGTH] > RXPACKET_MAX_LEN) ||
                    (rxpacket[PKT_ERROR] > 0x7F))
                {
                    rxpacket.erase(rxpacket.begin());
                    rx_length -= 1;
                    continue;
                }

                if (wait_length != (rxpacket[PKT_LENGTH] + PKT_LENGTH + 1))
                {
                    wait_length = rxpacket[PKT_LENGTH] + PKT_LENGTH + 1;
                    continue;
                }

                if (rx_length < wait_length)
                {
                    if (port.isPacketTimeout())
                    {
                        if (rx_length == 0)
                            result = scservo_def::COMM_RX_TIMEOUT;
                        else
                            result = scservo_def::COMM_RX_CORRUPT;

                        break;
                    }
                    else
                    {
                        continue;
                    }
                }

                // checksum
                checksum = 0;
                for (int i = 2; i < wait_length - 1; ++i)
                    checksum += rxpacket[i];

                checksum = (~checksum) & 0xFF;

                if (rxpacket[wait_length - 1] == checksum)
                    result = scservo_def::COMM_SUCCESS;
                else
                    result = scservo_def::COMM_RX_CORRUPT;

                break;
            }
            else
            {
                rxpacket.erase(rxpacket.begin(), rxpacket.begin() + idx);
                rx_length -= idx;
            }
        }
        else
        {
            if (port.isPacketTimeout())
            {
                if (rx_length == 0)
                    result = scservo_def::COMM_RX_TIMEOUT;
                else
                    result = scservo_def::COMM_RX_CORRUPT;

                break;
            }
        }
    }

    port.is_using = false;

    return {rxpacket, result};
}

std::tuple<uint32_t, int, uint8_t> protocol_packet_handler::read1ByteTxRx(PortHandler* port, uint8_t scs_id, uint16_t address)
{
    auto [data, result, error] = readTxRx(port, scs_id, address, 1);
    uint32_t data_read = (result == scservo_def::COMM_SUCCESS) ? data[0] : 0;
    return {data_read, result, error};
}

std::tuple<uint32_t, int, uint8_t> protocol_packet_handler::read2ByteTxRx(PortHandler* port, uint8_t scs_id, uint16_t address)
{
    auto [data, result, error] = readTxRx(port, scs_id, address, 2);
    uint32_t data_read = (result == scservo_def::COMM_SUCCESS) ? scservo_def::SCS_MAKEWORD(data[0], data[1]) : 0;
    return {data_read, result, error};
}

std::tuple<uint32_t, int, uint8_t> protocol_packet_handler::read4ByteTxRx(PortHandler* port, uint8_t scs_id, uint16_t address)
{
    auto [data, result, error] = readTxRx(port, scs_id, address, 4);
    uint32_t data_read = (result == scservo_def::COMM_SUCCESS) ? scservo_def::SCS_MAKEDWORD((data[0], data[1]), scservo_def::SCS_MAKEWORD(data[2], data[3])) : 0;
    return {data_read, result, error};
}

std::tuple<std::vector<uint8_t>, int, uint8_t> protocol_packet_handler::readTxRx(PortHandler* port, uint8_t scs_id, uint8_t address, uint8_t length)
{
    std::vector<uint8_t> txpacket(8, 0);
    std::vector<uint8_t> data;

    if (scs_id >= scservo_def::BROADCAST_ID)
    {
        return {data, scservo_def::COMM_NOT_AVAILABLE, 0};
    }

    txpacket[PKT_ID] = scs_id;
    txpacket[PKT_LENGTH] = 4;
    txpacket[PKT_INSTRUCTION] = scservo_def::INST_READ;
    txpacket[PKT_PARAMETER0 + 0] = address;
    txpacket[PKT_PARAMETER0 + 1] = length;

    auto [rxpacket, result, error] = txRxPacket(port, txpacket);

    if (result == scservo_def::COMM_SUCCESS)
    {
        error = rxpacket[PKT_ERROR];

        for (int i = 0; i < length; i++)
        {
            data.push_back(rxpacket[PKT_PARAMETER0 + i]);
        }
    }

    return {data, result, error};
}

std::tuple<std::vector<uint8_t>, int, uint8_t> protocol_packet_handler::txRxPacket(PortHandler* port, std::vector<uint8_t> txpacket)
{
    std::vector<uint8_t> rxpacket;
    uint8_t error = 0;

    // tx packet
    int result = txPacket(*port, txpacket);
    if (result != scservo_def::COMM_SUCCESS)
    {
        return {rxpacket, result, error};
    }

    // (ID == Broadcast ID) == no need to wait for status packet or not available
    if (txpacket[PKT_ID] == scservo_def::BROADCAST_ID)
    {
        port->is_using = false;
        return {rxpacket, result, error};
    }

    // set packet timeout
    if (txpacket[PKT_INSTRUCTION] == scservo_def::INST_READ)
    {
        port->setPacketTimeout(txpacket[PKT_PARAMETER0 + 1] + 6);
    }
    else
    {
        port->setPacketTimeout(6); // HEADER0 HEADER1 ID LENGTH ERROR CHECKSUM
    }

    // rx packet
    while (true)
    {
        auto [packet, rx_result] = rxPacket(*port);
        rxpacket = packet;
        result = rx_result;

        if (result != scservo_def::COMM_SUCCESS || txpacket[PKT_ID] == rxpacket[PKT_ID])
        {
            break;
        }
    }

    if (result == scservo_def::COMM_SUCCESS && txpacket[PKT_ID] == rxpacket[PKT_ID])
    {
        error = rxpacket[PKT_ERROR];
    }

    return {rxpacket, result, error};
}

std::string protocol_packet_handler::getTxRxResult(int result)
{
    if (result == scservo_def::COMM_SUCCESS)
        return "[TxRxResult] Communication success!";
    else if (result == scservo_def::COMM_PORT_BUSY)
        return "[TxRxResult] Port is in use!";
    else if (result == scservo_def::COMM_TX_FAIL)
        return "[TxRxResult] Failed transmit instruction packet!";
    else if (result == scservo_def::COMM_RX_FAIL)
        return "[TxRxResult] Failed get status packet from device!";
    else if (result == scservo_def::COMM_TX_ERROR)
        return "[TxRxResult] Incorrect instruction packet!";
    else if (result == scservo_def::COMM_RX_WAITING)
        return "[TxRxResult] Now receiving status packet!";
    else if (result == scservo_def::COMM_RX_TIMEOUT)
        return "[TxRxResult] There is no status packet!";
    else if (result == scservo_def::COMM_RX_CORRUPT)
        return "[TxRxResult] Incorrect status packet!";
    else if (result == scservo_def::COMM_NOT_AVAILABLE)
        return "[TxRxResult] Protocol does not support this function!";
    else
        return "";
}

std::string protocol_packet_handler::getRxPacketError(uint8_t error)
{
    if (error & ERRBIT_VOLTAGE)
        return "[RxPacketError] Input voltage error!";

    if (error & ERRBIT_ANGLE)
        return "[RxPacketError] Angle sen error!";

    if (error & ERRBIT_OVERHEAT)
        return "[RxPacketError] Overheat error!";

    if (error & ERRBIT_OVERELE)
        return "[RxPacketError] OverEle error!";
        
    if (error & ERRBIT_OVERLOAD)
        return "[RxPacketError] Overload error!";

    return "";
}

std::pair<int,int> protocol_packet_handler::writeTxRx(PortHandler* port, uint8_t scs_id, uint8_t address, uint8_t length, const std::vector<uint8_t>& data)
{
    std::vector<uint8_t> txpacket(length + 7, 0);

    txpacket[PKT_ID] = scs_id;
    txpacket[PKT_LENGTH] = length + 3;
    txpacket[PKT_INSTRUCTION] = scservo_def::INST_WRITE;
    txpacket[PKT_PARAMETER0] = address;

    std::copy(
        data.begin(),
        data.begin() + length,
        txpacket.begin() + PKT_PARAMETER0 + 1
    );

    auto [rxpacket, result, error] = txRxPacket(port, txpacket);

    return {result, error};
}

int protocol_packet_handler::syncReadTx(
    PortHandler& port,
    int start_address,
    int data_length,
    const std::vector<uint8_t>& param,
    int param_length)
{
    std::vector<uint8_t> txpacket(param_length + 8, 0);

    // HEADER0 HEADER1 ID LEN INST START_ADDR DATA_LEN CHKSUM

    txpacket[PKT_ID] = scservo_def::BROADCAST_ID;
    txpacket[PKT_LENGTH] = param_length + 4;
    txpacket[PKT_INSTRUCTION] = scservo_def::INST_SYNC_READ;

    txpacket[PKT_PARAMETER0 + 0] = static_cast<uint8_t>(start_address);
    txpacket[PKT_PARAMETER0 + 1] = static_cast<uint8_t>(data_length);

    for (int i = 0; i < param_length; ++i)
    {
        txpacket[PKT_PARAMETER0 + 2 + i] = param[i];
    }

    int result = txPacket(port, txpacket);

    if (result == scservo_def::COMM_SUCCESS)
    {
        port.setPacketTimeout((6 + data_length) * param_length);
    }

    return result;
}

std::tuple<std::vector<uint8_t>, int, uint8_t> protocol_packet_handler::readRx(PortHandler* port, uint8_t scs_id, uint8_t length)
{
    int result = scservo_def::COMM_TX_FAIL;
    uint8_t error = 0;

    std::vector<uint8_t> rxpacket;
    std::vector<uint8_t> data;

    while (true)
    {
        auto [packet, res] = rxPacket(*port);
        rxpacket = packet;
        result = res;

        if (result != scservo_def::COMM_SUCCESS || rxpacket[PKT_ID] == scs_id)
            break;
    }

    if (result == scservo_def::COMM_SUCCESS && rxpacket[PKT_ID] == scs_id)
    {
        error = rxpacket[PKT_ERROR];

        for (int i = 0; i < length; ++i)
        {
            data.push_back(rxpacket[PKT_PARAMETER0 + i]);
        }
    }

    return {data, result, error};
}

int protocol_packet_handler::syncWriteTxOnly(
    PortHandler * port,
    int start_address,
    int data_length,
    const std::vector<uint8_t> & param,
    int param_length
)
{
    std::vector<uint8_t> txpacket(param_length + 8, 0);
    // 8: HEADER0 HEADER1 ID LEN INST START_ADDR DATA_LEN ... CHKSUM

    txpacket[PKT_ID] = scservo_def::BROADCAST_ID;
    txpacket[PKT_LENGTH] = static_cast<uint8_t>(param_length + 4);
    txpacket[PKT_INSTRUCTION] = scservo_def::INST_SYNC_WRITE;
    txpacket[PKT_PARAMETER0 + 0] = static_cast<uint8_t>(start_address);
    txpacket[PKT_PARAMETER0 + 1] = static_cast<uint8_t>(data_length);

    for (int i = 0; i < param_length; ++i) {
        txpacket[PKT_PARAMETER0 + 2 + i] = param[i];
    }

    uint8_t error = 0;
    int result = scservo_def::COMM_TX_FAIL;

    std::tie(std::ignore, result, error) = txRxPacket(port, txpacket);

    return result;
}