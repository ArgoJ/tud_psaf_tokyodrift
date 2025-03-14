#include <string_view>
#include <chrono>
#include <string>
#include <memory>
#include <utility>
#include "psaf_ucbridge/communication/data_receiver.hpp"
#include "psaf_ucbridge/configuration/static_config.hpp"

DataReceiver::DataReceiver(std::shared_ptr<MessageQueue<std::string>> dataQueue)
: UcBridgeThread(std::chrono::milliseconds(1)),
  display_txt_(nullptr),
  dataQueue(std::move(dataQueue)) {}

void DataReceiver::doWork()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::string message;
  if (!dataQueue->pop(message) && message.length() < 2) {
    return;
  }
  if (message[0] == CommandElement::command_element().kTextMessagePrefix) {
    if (display_txt_ != nullptr) {
      display_txt_->publishData(message);
    } else {
      return;
    }
  } else {
    // find group number
    int colon = message.find(':');
    int groupNumber = std::stoi(message.substr(1, colon - 1));
    if (sensorGroups.find(groupNumber) != sensorGroups.end()) {
      sensorGroups.at(groupNumber)->publishData(message);
    }
  }
}

void DataReceiver::registerSensorGroup(SensorGroup::UniquePtr group)
{
  std::lock_guard<std::mutex> lock(mutex_);
  sensorGroups.insert({group->getSensorGroupNo(), std::move(group)});
}

void DataReceiver::unregisterSensorGroups()
{
  sensorGroups.clear();
}
bool DataReceiver::unregisterSensorGroup(int key)
{
  auto it = sensorGroups.find(key);
  if (it != sensorGroups.end()) {
    sensorGroups.erase(it);
    return true;
  } else {return false;}
}
