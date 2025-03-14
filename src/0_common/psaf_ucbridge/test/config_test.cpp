#include <string>
#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"
#include "psaf_ucbridge/configuration/configuration.hpp"
class TestNode : public rclcpp::Node
{
public:
  TestNode()
  : Node("Mock_node")
  {
    this->declare_parameter("sensor_groups.us.activate", false);
    this->declare_parameter("sensor_groups.imu.activate", false);
    this->declare_parameter("sensor_groups.mag.activate", false);
    this->declare_parameter("sensor_groups.enc_step.activate", false);
    this->declare_parameter("sensor_groups.enc_cm.activate", false);
    this->declare_parameter("sensor_groups.hall_dt.activate", false);
    this->declare_parameter("sensor_groups.hall_dt8.activate", false);
    this->declare_parameter("general.raw_communication", false);
    this->declare_parameter("sensor_groups.imu.ts", 88);
    this->declare_parameter("serial_port_configuration.serial_port", "/dev/ttyUSB0");
    this->declare_parameter("general.log_level", "debug");
    this->declare_parameter("imu_sensor.acceleration", 16);
    this->declare_parameter("imu_sensor.rotation", 1000);
    this->declare_parameter("imu_sensor.afilt", 4);
    this->declare_parameter("imu_sensor.gfilt", 4);
    this->declare_parameter("mag_sensor.useasa", false);
    this->declare_parameter("sensor_groups.imu.group_num", -1);
    this->declare_parameter("topic_name.led", "kkk");
    this->declare_parameter("sensor_groups.enc_step.skip", 20);
    this->declare_parameter("sensor_groups.enc_cm.skip", 20);
  }
};

/**
 * @brief Tests Default configuration
 */
TEST(test_configuration, test_default) {
  Configuration & config = Configuration::instance();
  ASSERT_EQ(config.serialPort(), "/dev/ttyUSB0");
  ASSERT_EQ(config.logLevel(), RCUTILS_LOG_SEVERITY_INFO);
  ASSERT_EQ(config.imuTs(), 10);
  ASSERT_EQ(config.encStepSkip(), 5);
  ASSERT_EQ(config.encCmSkip(), 5);
  ASSERT_EQ(config.activateImuSensorGroup(), true);
  ASSERT_EQ(config.activateHallDt8SensorGroup(), true);
  ASSERT_EQ(config.activateHallDtSensorGroup(), true);
  ASSERT_EQ(config.activateMagSensorGroup(), true);
  ASSERT_EQ(config.activateRawCommunication(), false);
  ASSERT_EQ(config.activateUSSensorGroup(), true);
  ASSERT_EQ(config.activateEncStepSensorGroup(), true);
  ASSERT_EQ(config.activateEncCmSensorGroup(), true);
  ASSERT_EQ(config.kImuAcceleration, CommandElement::IMUAcceleration::ImuAcc4);
  ASSERT_EQ(config.kImuRotationRate, CommandElement::IMURotationRate::ImuRot500);
  ASSERT_EQ(config.kImuAFilt, CommandElement::IMUAFiltMode::AFILT0);
  ASSERT_EQ(config.kImuGFilt, CommandElement::IMUGFiltMode::GFILT0);
  ASSERT_TRUE(config.kMagUseasa);
  ASSERT_EQ(config.kUsRange, 100);
  ASSERT_EQ(config.kUsGain, 10);
  ASSERT_EQ(config.kImuSensorGroupNo, 0);
  ASSERT_EQ(config.kUSSensorGroupNo, 1);
  ASSERT_EQ(config.kMagSensorGroupNo, 2);
  ASSERT_EQ(config.kHallDtSensorGroupNo, 3);
  ASSERT_EQ(config.kHallDt8SensorGroupNo, 4);
  ASSERT_EQ(config.kTopicDtData, "dt_data");
  ASSERT_EQ(config.kTopicDt8Data, "dt8_data");
  ASSERT_EQ(config.kTopicHallCntData, "hall_cnt_data");
  ASSERT_EQ(config.kTopicImuData, "imu_data");
  ASSERT_EQ(config.kTopicMagData, "mag_data");
  ASSERT_EQ(config.kTopicUsRangeSuffix, "_range");
  ASSERT_EQ(config.kTopicSetMotorLevelForward, "uc_bridge/set_motor_level_forward");
  ASSERT_EQ(config.kTopicSetMotorLevelBackward, "uc_bridge/set_motor_level_backward");
  ASSERT_EQ(config.kTopicSetLed, "uc_bridge/light");
  ASSERT_EQ(config.kTopicEncStepData, "enc_step_data");
  ASSERT_EQ(config.kTopicEncCmData, "enc_cm_data");
}

/**
 * @brief Tests Node declared Parameters
 */
TEST(test_configuration, test_readNodeConfig) {
  rclcpp::init(0, nullptr);
  TestNode mock_node;
  Configuration & config = Configuration::instance();
  config.readConfigurationFromNode(mock_node);
  ASSERT_EQ(config.activateUSSensorGroup(), false);
  ASSERT_EQ(config.activateRawCommunication(), false);
  ASSERT_EQ(config.activateMagSensorGroup(), false);
  ASSERT_EQ(config.activateHallDtSensorGroup(), false);
  ASSERT_EQ(config.activateHallDt8SensorGroup(), false);
  ASSERT_EQ(config.activateImuSensorGroup(), false);
  ASSERT_EQ(config.activateEncStepSensorGroup(), false);
  ASSERT_EQ(config.activateEncCmSensorGroup(), false);
  ASSERT_EQ(config.imuTs(), 88);
  ASSERT_EQ(config.encStepSkip(), 20);
  ASSERT_EQ(config.encCmSkip(), 20);
  ASSERT_EQ(config.serialPort(), "/dev/ttyUSB0");
  ASSERT_EQ(config.logLevel(), RCUTILS_LOG_SEVERITY_DEBUG);
  ASSERT_EQ(config.kImuAcceleration, CommandElement::IMUAcceleration::ImuAcc16);
  ASSERT_EQ(config.kImuRotationRate, CommandElement::IMURotationRate::ImuRot1000);
  ASSERT_EQ(config.kImuAFilt, CommandElement::IMUAFiltMode::AFILT4);
  ASSERT_EQ(config.kImuGFilt, CommandElement::IMUGFiltMode::GFILT4);
  ASSERT_FALSE(config.kMagUseasa);
  ASSERT_EQ(config.kUsRange, 100);
  ASSERT_EQ(config.kUsGain, 10);
  ASSERT_EQ(config.kImuSensorGroupNo, 0);
  ASSERT_EQ(config.kUSSensorGroupNo, 1);
  ASSERT_EQ(config.kMagSensorGroupNo, 2);
  ASSERT_EQ(config.kHallDtSensorGroupNo, 3);
  ASSERT_EQ(config.kHallDt8SensorGroupNo, 4);
  ASSERT_EQ(config.kTopicDtData, "dt_data");
  ASSERT_EQ(config.kTopicDt8Data, "dt8_data");
  ASSERT_EQ(config.kTopicHallCntData, "hall_cnt_data");
  ASSERT_EQ(config.kTopicImuData, "imu_data");
  ASSERT_EQ(config.kTopicMagData, "mag_data");
  ASSERT_EQ(config.kTopicUsRangeSuffix, "_range");
  ASSERT_EQ(config.kTopicSetMotorLevelForward, "uc_bridge/set_motor_level_forward");
  ASSERT_EQ(config.kTopicSetMotorLevelBackward, "uc_bridge/set_motor_level_backward");
  ASSERT_EQ(config.kTopicSetLed, "kkk");

  rclcpp::shutdown();
}
