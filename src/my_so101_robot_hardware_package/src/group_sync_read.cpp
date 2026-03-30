#include "my_so101_robot_hardware_package/group_sync_read.hpp"
#include "my_so101_robot_hardware_package/port_handler.hpp"
#include "my_so101_robot_hardware_package/protocol_packet_handler.hpp"

GroupSyncRead::GroupSyncRead(PortHandler* port, protocol_packet_handler* ph, int start_address, int data_length)
    : port_(port),
      ph_(ph),
      start_address_(start_address),
      data_length_(data_length),
      last_result(false),
      is_param_changed(false),
      param_{},
      data_dict_{}
{
    clearParam();
}

void GroupSyncRead::clearParam()
{
    data_dict_.clear();
}

int GroupSyncRead::txRxPacket()
{
    int result = txPacket();

    if (result != scservo_def::COMM_SUCCESS)
        return result;

    return rxPacket();
}

int GroupSyncRead::txPacket()
{
    if (data_dict_.empty())
        return scservo_def::COMM_NOT_AVAILABLE;

    if (is_param_changed || param_.empty())
        makeParam();

    return ph_->syncReadTx(
        *port_,
        start_address_,
        data_length_,
        param_,
        data_dict_.size()
    );
}

int GroupSyncRead::rxPacket()
{
    last_result = false;

    int result = scservo_def::COMM_RX_FAIL;

    if (data_dict_.empty())
        return scservo_def::COMM_NOT_AVAILABLE;

    for (auto& [scs_id, value] : data_dict_)
    {
        auto [data, res, err] = ph_->readRx(port_, scs_id, data_length_);

        if (res != scservo_def::COMM_SUCCESS)
            return res;

        value = data;
        result = res;
    }

    if (result == scservo_def::COMM_SUCCESS)
        last_result = true;

    return result;
}

void GroupSyncRead::makeParam()
{
    if (data_dict_.empty())
        return;

    param_.clear();

    for (const auto& [scs_id, _] : data_dict_)
    {
        param_.push_back(static_cast<uint8_t>(scs_id));
    }
}

bool GroupSyncRead::addParam(int scs_id)
{
    if (data_dict_.find(scs_id) != data_dict_.end())
        return false;

    data_dict_[scs_id] = {};
    is_param_changed = true;
    return true;
}

int GroupSyncRead::getData(int scs_id, int address, int data_length)
{
    if (!isAvailable(scs_id, address, data_length))
        return 0;

    int index = address - start_address_;
    const auto& data = data_dict_.at(scs_id);

    if (data_length == 1)
    {
        return data[index];
    }
    else if (data_length == 2)
    {
        return scservo_def::SCS_MAKEWORD(
            data[index],
            data[index + 1]);
    }
    else if (data_length == 4)
    {
        return scservo_def::SCS_MAKEDWORD(
            scservo_def::SCS_MAKEWORD(data[index],     data[index + 1]),
            scservo_def::SCS_MAKEWORD(data[index + 2], data[index + 3]));
    }

    return 0;
}

bool GroupSyncRead::isAvailable(int scs_id, int address, int data_length)
{
    if (data_dict_.find(scs_id) == data_dict_.end())
        return false;

    if ((address < start_address_) ||
        (start_address_ + data_length_ - data_length < address))
        return false;

    if (data_dict_.at(scs_id).size() < static_cast<size_t>(data_length))
        return false;

    return true;
}