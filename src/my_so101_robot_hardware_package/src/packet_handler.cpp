#include "my_so101_robot_hardware_package/packet_handler.hpp"

protocol_packet_handler PacketHandler(int protocol_end) {
    scservo_def::SCS_SETEND(protocol_end);
    return protocol_packet_handler();
}