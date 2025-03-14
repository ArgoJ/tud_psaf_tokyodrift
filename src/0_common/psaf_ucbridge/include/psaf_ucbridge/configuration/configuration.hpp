#ifndef PSAF_UCBRIDGE__CONFIGURATION_HPP_
#define PSAF_UCBRIDGE__CONFIGURATION_HPP_
#include <unordered_map>
#include <string>
#include "psaf_ucbridge/configuration/dynamic_config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "psaf_ucbridge/configuration/configuration_item_base.hpp"
#include "psaf_ucbridge/configuration/configuration_item.hpp"
#include "psaf_ucbridge/configuration/configuration_interfaces.hpp"

class Configuration
  : public ConfigurationBase,
  public ISerialPortConfiguration,
  public ISensorGroupConfiguration,
  public IGeneralConfiguration,
  public DynamicConfig
{
public:
  /**
   * Delete all copy/move constructors and copy/move assignments.
   * Class should be used as singleton.
   */
  ~Configuration() = default;
  Configuration(const Configuration &) = delete;
  Configuration & operator=(const Configuration &) = delete;

private:
  Configuration();

public:
  static Configuration & instance();

  // configuration for serial port
  std::string & serialPort() override;

  // configuration for sensor groups
  bool activateTxtDisplay() override;

  bool activateImuSensorGroup() override;

  bool activateUSSensorGroup() override;

  bool activateMagSensorGroup() override;

  bool activateHallDtSensorGroup() override;

  bool activateHallDt8SensorGroup() override;

  bool activateEncStepSensorGroup() override;

  bool activateEncCmSensorGroup() override;

  bool activateRawCommunication() override;

  bool activatePbSensorGroup() override;

  bool activateVbatSensorGroup() override;

  bool activateUnkGrp() override;

  bool activateRc() override;

  int logLevel() override;

  int board_version() override;

  bool reset_start() override;

  bool reset_end() override;

  int rc_frequency() override;

  int steer_frequency() override;

  int imuTs() override;

  int encStepSkip() override;

  int encCmSkip() override;

  int deadMan() override;


  void readConfigurationFromNode(rclcpp::Node & node);

private:
  std::unordered_map<std::string, int> logLevelMapping;

  std::unordered_map<int, CommandElement::IMUAcceleration> imuAccMapping;

  std::unordered_map<int, CommandElement::IMURotationRate> imuRotMapping;

  std::unordered_map<int, CommandElement::IMUAFiltMode> imuAfiltMapping;

  std::unordered_map<int, CommandElement::IMUGFiltMode> imuGfiltMapping;

  void initializeSerialPortConfiguration();

  void initializeSensorGroupConfiguration();

  void initializeGeneralConfiguration();

  template<class T>
  void importValueFromNode(
    rclcpp::Node & node,
    std::string && id,
    std::string && parameterName,
    T && defaultValue)
  {
    T result;
    // check if parameter was declared
    if (!node.has_parameter(parameterName)) {
      node.declare_parameter(parameterName, defaultValue);
    }
    node.get_parameter<T>(parameterName, result);
    this->insertValue(id, result);
  }

  template<class T>
  T getValueFromNode(
    rclcpp::Node & node,
    std::string && parameterName,
    T const && defaultValue
  )
  {
    T result;
    // check if parameter was declared
    if (!node.has_parameter(parameterName)) {
      node.declare_parameter(parameterName, defaultValue);
    }
    node.get_parameter<T>(parameterName, result);
    return result;
  }

  template<class T>
  T getValueFromNode(
    rclcpp::Node & node,
    std::string && parameterName,
    T const & defaultValue
  )
  {
    T result;
    // check if parameter was declared
    if (!node.has_parameter(parameterName)) {
      node.declare_parameter(parameterName, defaultValue);
    }
    node.get_parameter<T>(parameterName, result);
    return result;
  }

  template<typename T1, typename T2>
  void changeImuDynamicFromNode(
    rclcpp::Node & node,
    std::string && parameterName,
    std::unordered_map<int, T2> & map,
    T1 & defaultValue
  )
  {
    if (!node.has_parameter(parameterName)) {
      node.declare_parameter(parameterName, static_cast<int>(defaultValue));
    } else {
      int result;
      node.get_parameter(parameterName, result);
      if (result != static_cast<int>(defaultValue)) {
        if (map.find(result) != map.end()) {
          defaultValue = map[result];
        } else {
          RCLCPP_INFO(
            node.get_logger(),
            "Invalid %s value, using default!",
            parameterName.c_str());
        }
      }
    }
  }
};

#endif  // PSAF_UCBRIDGE__CONFIGURATION_HPP_
