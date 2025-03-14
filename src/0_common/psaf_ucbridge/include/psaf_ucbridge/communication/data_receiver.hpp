#ifndef PSAF_UCBRIDGE_DATARECEIVER_HPP
#define PSAF_UCBRIDGE_DATARECEIVER_HPP

#include <memory>
#include <map>
#include <string>
#include <utility>
#include "psaf_ucbridge/communication/message_queue.hpp"
#include "psaf_ucbridge/sensor_groups/sensor_group.hpp"
#include "psaf_ucbridge/sensor_groups/display_txt.hpp"
#include "psaf_ucbridge/uc_bridge_thread.hpp"

class DataReceiver : public UcBridgeThread
{
public:
  /**
   *
   * @param dataQueue
   */
  explicit DataReceiver(std::shared_ptr<MessageQueue<std::string>> dataQueue);

  /**
   *
   * @param group
   */
  void registerSensorGroup(SensorGroup::UniquePtr group);

  void registerDisplayTxt(DisplayTxt::UniquePtr dis)
  {
    display_txt_ = std::move(dis);
  }

  /**
   *
   */
  void unregisterSensorGroups();

  bool unregisterSensorGroup(int key);

protected:
  /**
    * @brief Worker function, responsible for polling the serial input buffer.
    */
  void doWork() final;

private:
  /**
   *
   */
  std::map<int, SensorGroup::UniquePtr> sensorGroups;

  std::mutex mutex_;

  DisplayTxt::UniquePtr display_txt_;

  /**
   *
   */
  std::shared_ptr<MessageQueue<std::string>> dataQueue;
};

#endif  // PSAF_UCBRIDGE_DATARECEIVER_HPP
