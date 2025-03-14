#ifndef PSAF_UCBRIDGE__CONFIGURATION_INTERFACES_HPP_
#define PSAF_UCBRIDGE__CONFIGURATION_INTERFACES_HPP_
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "psaf_ucbridge/configuration/configuration_base.hpp"
#include "psaf_ucbridge/configuration/default_configuration.hpp"
#include "psaf_ucbridge/logging/log_level.hpp"

class ISerialPortConfiguration
{
public:
  using SharedPtr = std::shared_ptr<ISerialPortConfiguration>;

  virtual std::string & serialPort() = 0;
};

class ISensorGroupConfiguration
{
public:
  using SharedPtr = std::shared_ptr<ISensorGroupConfiguration>;

  virtual bool activateTxtDisplay() = 0;

  virtual bool activateImuSensorGroup() = 0;

  virtual bool activateUSSensorGroup() = 0;

  virtual bool activateMagSensorGroup() = 0;

  virtual bool activateHallDtSensorGroup() = 0;

  virtual bool activateHallDt8SensorGroup() = 0;

  virtual bool activateEncStepSensorGroup() = 0;

  virtual bool activateEncCmSensorGroup() = 0;

  virtual bool activatePbSensorGroup() = 0;

  virtual bool activateVbatSensorGroup() = 0;

  virtual bool activateUnkGrp() = 0;

  virtual bool activateRc() = 0;

  virtual int imuTs() = 0;

  virtual int encStepSkip() = 0;

  virtual int encCmSkip() = 0;

  virtual int deadMan() = 0;

  virtual bool reset_start() = 0;

  virtual bool reset_end() = 0;
};

class IGeneralConfiguration
{
public:
  IGeneralConfiguration()
  : kTopicRawCommunication(DEFAULT_TOPIC_RAW_COMMUNICATION) {}

  using SharedPtr = std::shared_ptr<IGeneralConfiguration>;

  const std::string kTopicRawCommunication;

  virtual bool activateRawCommunication() = 0;

  virtual int logLevel() = 0;

  virtual int board_version() = 0;

  virtual int rc_frequency() = 0;

  virtual int steer_frequency() = 0;
};

#endif  // PSAF_UCBRIDGE__CONFIGURATION_INTERFACES_HPP_
