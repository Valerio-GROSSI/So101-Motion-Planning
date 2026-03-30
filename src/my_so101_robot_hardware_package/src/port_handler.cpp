#include "my_so101_robot_hardware_package/port_handler.hpp"

PortHandler::PortHandler(const std::string& port_name)
    : is_open(false),
      baudrate_(port_handler_const::DEFAULT_BAUDRATE),
      packet_start_time(0.0),
      packet_timeout(0.0),
      tx_time_per_byte(0.0),
      is_using(false),
      port_name_(port_name)
{};

bool PortHandler::openPort() {
    return setBaudRate(baudrate_);
}

bool PortHandler::setBaudRate(int baudrate) {
  int baud = getCFlagBaud(baudrate);

  if (baud <= 0){
    return false;
  }
  else {
    baudrate_ = baudrate;
    return setupPort(baud);
  }
}

int PortHandler::getCFlagBaud(int baudrate) {
  switch (baudrate)
    {
        case 4800:
        case 9600:
        case 14400:
        case 19200:
        case 38400:
        case 57600:
        case 115200:
        case 128000:
        case 250000:
        case 500000:
        case 1000000:
            return baudrate;

        default:
            return -1;
    }
}

bool PortHandler::setupPort(int cflag_baud)
{
    if (is_open)
        closePort();

    try
    {
        ser.setPort(port_name_);
        ser.setBaudrate(baudrate_);

        serial::Timeout timeout = serial::Timeout::simpleTimeout(0);
        ser.setTimeout(timeout);

        ser.open();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Serial open error: " << e.what() << std::endl;
        return false;
    }

    if (!ser.isOpen())
        return false;

    is_open = true;

    ser.flushInput();   // équivalent reset_input_buffer()

    tx_time_per_byte = (1000.0 / baudrate_) * 10.0;

    return true;
}

void PortHandler::closePort() {
    ser.close();
    is_open = false;
}

void PortHandler::setPacketTimeoutMillis(int msec)
{
    packet_start_time = getCurrentTime();
    packet_timeout = static_cast<double>(msec);
}

double PortHandler::getCurrentTime()
{
    using namespace std::chrono;

    auto now = high_resolution_clock::now();
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()).count();

    return static_cast<double>(ms);
}

void PortHandler::setPacketTimeout(int packet_length)
{
    packet_start_time = getCurrentTime();
    packet_timeout = (tx_time_per_byte * packet_length) + (tx_time_per_byte * 3.0) + port_handler_const::LATENCY_TIMER;
}

void PortHandler::clearPort() {
    ser.flush();
}

size_t PortHandler::writePort(std::vector<uint8_t>& packet) {
    return ser.write(packet.data(), packet.size());
}

std::vector<uint8_t> PortHandler::readPort(int length)
{
    std::vector<uint8_t> buffer(length);

    size_t bytes_read = ser.read(buffer.data(), length);

    buffer.resize(bytes_read);

    return buffer;
}

bool PortHandler::isPacketTimeout()
{
    if (getTimeSinceStart() > packet_timeout)
    {
        packet_timeout = 0;
        return true;
    }

    return false;
}

double PortHandler::getTimeSinceStart()
{
    double time_since = getCurrentTime() - packet_start_time;

    if (time_since < 0.0)
    {
        packet_start_time = getCurrentTime();
    }

    return time_since;
}


// PacketHandler::PacketHandler(int protocol_version) {
// };

// void PortHandler::closePort() {
//     ser.close();
//     is_open_ = false;
// }

// bool PortHandler::setupPort(int /*cflag_baud*/) {
//     if (is_open_) {
//         closePort();
//     }

//     boost::system::error_code ec;
//     ser_.open(port_name_, ec);
//     if (ec) {
//         is_open_ = false;
//         return false;
//     }

//     // Configuration minimale proche de pyserial:
//     // bytesize=8, parity none, stopbits 1, timeout=0 (non-bloquant)
//     ser_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_), ec);
//     ser_.set_option(boost::asio::serial_port_base::character_size(8), ec);
//     ser_.set_option(boost::asio::serial_port_base::parity(
//         boost::asio::serial_port_base::parity::none), ec);
//     ser_.set_option(boost::asio::serial_port_base::stop_bits(
//         boost::asio::serial_port_base::stop_bits::one), ec);
//     ser_.set_option(boost::asio::serial_port_base::flow_control(
//         boost::asio::serial_port_base::flow_control::none), ec);

//     if (ec) {
//         closePort();
//         return false;
//     }

//     is_open_ = true;

//     // "reset_input_buffer" : on vide ce qu'il y a déjà
//     clearPort();

//     // Python: (1000.0 / baudrate) * 10.0  -> ms/byte pour 10 bits (start+8data+stop)
//     tx_time_per_byte_ms_ = (1000.0 / static_cast<double>(baudrate_)) * 10.0;

//     return true;
// }