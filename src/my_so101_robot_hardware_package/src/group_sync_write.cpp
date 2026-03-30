#include "my_so101_robot_hardware_package/group_sync_write.hpp"
#include "my_so101_robot_hardware_package/protocol_packet_handler.hpp"

GroupSyncWrite::GroupSyncWrite(
    PortHandler * port,
    protocol_packet_handler * ph,
    int start_address,
    int data_length
)
: port_(port),
  ph_(ph),
  start_address_(start_address),
  data_length_(data_length),
  is_param_changed(false)
{
    clearParam();
}

void GroupSyncWrite::makeParam()
{
    if (data_dict.empty()) {
        return;
    }

    param.clear();

    for (const auto & [scs_id, data] : data_dict) {
        if (data.empty()) {
            return;
        }

        param.push_back(scs_id);
        param.insert(param.end(), data.begin(), data.end());
    }
}

bool GroupSyncWrite::addParam(uint8_t scs_id, const std::vector<uint8_t> & data)
{
    if (data_dict.find(scs_id) != data_dict.end()) {
        // scs_id already exists
        return false;
    }

    if (static_cast<int>(data.size()) > data_length_) {
        // input data is longer than allowed
        return false;
    }

    data_dict[scs_id] = data;
    is_param_changed = true;
    return true;
}

void GroupSyncWrite::removeParam(uint8_t scs_id)
{
    auto it = data_dict.find(scs_id);
    if (it == data_dict.end()) {
        // does not exist
        return;
    }

    data_dict.erase(it);
    is_param_changed = true;
}

bool GroupSyncWrite::changeParam(uint8_t scs_id, const std::vector<uint8_t> & data)
{
    auto it = data_dict.find(scs_id);
    if (it == data_dict.end()) {
        // does not exist
        return false;
    }

    if (static_cast<int>(data.size()) > data_length_) {
        // input data is longer than allowed
        return false;
    }

    it->second = data;
    is_param_changed = true;
    return true;
}

void GroupSyncWrite::clearParam()
{
    data_dict.clear();
    //param.clear();
    //is_param_changed = false;
}

int GroupSyncWrite::txPacket()
{
    if (data_dict.empty()) {
        return scservo_def::COMM_NOT_AVAILABLE;
    }

    if (is_param_changed || param.empty()) {
        makeParam();
        //is_param_changed = false;
    }

    return ph_->syncWriteTxOnly(
        port_,
        start_address_,
        data_length_,
        param,
        static_cast<int>(data_dict.size()) * (1 + data_length_)
    );
}