#include <string>
#include <unordered_set>
#include "rclcpp/rclcpp.hpp"
#include "psaf_ucbridge/configuration/configuration.hpp"
#include "psaf_ucbridge/configuration/default_configuration.hpp"

Configuration::Configuration()
{
  initializeSerialPortConfiguration();
  initializeSensorGroupConfiguration();
  initializeGeneralConfiguration();
}

Configuration & Configuration::instance()
{
  static Configuration _instance;
  return _instance;
}

void Configuration::readConfigurationFromNode(rclcpp::Node & node)
{
  importValueFromNode(
    node,
    "serialPort",
    "serial_port_configuration.serial_port",
    DEFAULT_SERIAL_PORT);

  importValueFromNode(
    node,
    "activateTxtDisplay",
    "general.display_txt",
    DEFAULT_ACTIVATE_TXT_DISPLAY
  );

  importValueFromNode(
    node,
    "rcFrequency",
    "general.rc_frequency",
    DEFAULT_RC_FREQUENCY
  );

  importValueFromNode(
    node,
    "steerFrequency",
    "general.steer_frequency",
    DEFAULT_STEER_FREQUENCY
  );

  importValueFromNode(
    node,
    "boardVersion",
    "general.board_version",
    DEFAULT_BOARD_VERSION
  );

  importValueFromNode(
    node,
    "reset_start",
    "general.reset_ucboard_at_start",
    DEFAULT_RESET_START
  );

  importValueFromNode(
    node,
    "reset_end",
    "general.reset_ucboard_at_stop",
    DEFAULT_RESET_END
  );

  importValueFromNode(
    node,
    "activateUnkGrp",
    "sensor_groups.unkgrp.activate",
    DEFAULT_ACTIVATE_UNK_GRP);

  importValueFromNode(
    node,
    "activateRc",
    "sensor_groups.rc.activate",
    DEFAULT_ACTIVATE_RC
  );

  importValueFromNode(
    node,
    "activateImuSensorGroup",
    "sensor_groups.imu.activate",
    DEFAULT_ACTIVATE_IMU_SENSOR_GROUP
  );

  importValueFromNode(
    node,
    "activateUSSensorGroup",
    "sensor_groups.us.activate",
    DEFAULT_ACTIVATE_US_SENSOR_GROUP
  );

  importValueFromNode(
    node,
    "activateMagSensorGroup",
    "sensor_groups.mag.activate",
    DEFAULT_ACTIVATE_MAG_SENSOR_GROUP
  );

  importValueFromNode(
    node,
    "activateHallDtSensorGroup",
    "sensor_groups.hall_dt.activate",
    DEFAULT_ACTIVATE_HALL_DT_SENSOR_GROUP
  );

  importValueFromNode(
    node,
    "activateHallDt8SensorGroup",
    "sensor_groups.hall_dt8.activate",
    DEFAULT_ACTIVATE_HALL_DT8_SENSOR_GROUP
  );

  importValueFromNode(
    node,
    "activateEncStepSensorGroup",
    "sensor_groups.enc_step.activate",
    DEFAULT_ACTIVATE_ENC_STEP_SENSOR_GROUP
  );

  importValueFromNode(
    node,
    "activateEncCmSensorGroup",
    "sensor_groups.enc_cm.activate",
    DEFAULT_ACTIVATE_ENC_CMS_SENSOR_GROUP
  );

  importValueFromNode(
    node,
    "activateRawCommunication",
    "general.raw_communication",
    DEFAULT_ACTIVATE_RAW_COMMUNICATION);

  importValueFromNode(
    node,
    "activatePbSensorGroup",
    "sensor_group.vbat.activate",
    DEFAULT_ACTIVATE_PB_SENSOR_GROUP
  );

  importValueFromNode(
    node,
    "activateVbatSensorGroup",
    "sensor_group.vbat.activate",
    DEFAULT_ACTIVATE_VBAT_SENSOR_GROUP
  );

  importValueFromNode(
    node,
    "imuTs",
    "sensor_groups.imu.ts",
    DEFAULT_IMU_TS
  );

  importValueFromNode(
    node,
    "encStepSkip",
    "sensor_groups.enc_step.skip",
    DEFAULT_ENC_STEP_SKIP
  );

  importValueFromNode(
    node,
    "encCmSkip",
    "sensor_groups.enc_cm.skip",
    DEFAULT_ENC_CM_SKIP
  );

  importValueFromNode(
    node,
    "deadMan",
    "drive.dead_man_switch_interval",
    DEFAULT_DEAD_MAN);

  std::string logLevel = getValueFromNode(node, "general.log_level", DEFAULT_LOG_LEVEL);
  if (logLevelMapping.find(logLevel) != logLevelMapping.end()) {
    int val = logLevelMapping[logLevel];
    insertValue<int>("logLevel", val);
  } else {
    int val = logLevelMapping[DEFAULT_LOG_LEVEL];
    insertValue<int>("logLevel", val);
  }

  /**
   * IMU Acceleration
   */
  changeImuDynamicFromNode(node, "imu_sensor.acceleration", imuAccMapping, kImuAcceleration);

  /**
   * IMU Rotation rate
   */
  changeImuDynamicFromNode(node, "imu_sensor.rotation", imuRotMapping, kImuRotationRate);

  /**
   * IMU AFilt
   */
  changeImuDynamicFromNode(node, "imu_sensor.afilt", imuAfiltMapping, kImuAFilt);

  /**
   * IMU Gfilt
   */
  changeImuDynamicFromNode(node, "imu_sensor.gfilt", imuGfiltMapping, kImuGFilt);

  /**
   * MAG Useasa
   */
  {
    bool result = getValueFromNode(node, "mag_sensor.useasa", kMagUseasa);
    if (result != kMagUseasa) {kMagUseasa = result;}
  }

  /**
   * US Range Gain
   */
  {
    // rclcpp don't support uint_32t as parameter
    int range = static_cast<int>(kUsRange);
    range = getValueFromNode(node, "us_sensor.range", range);
    if (range < 0) {
      RCLCPP_INFO(node.get_logger(), "Invalid us_sensor.range! Using default!");
    } else {
      if (static_cast<uint32_t>(range) != kUsRange) {kUsRange = range;}
    }

    int gain = static_cast<int>(kUsGain);
    gain = getValueFromNode(node, "us_sensor.gain", gain);
    gain = getValueFromNode(node, "us_sensor.gain", gain);
    if (gain < 0) {RCLCPP_INFO(node.get_logger(), "Invalid us_sensor.gain! Using default!");} else {
      if (static_cast<uint32_t>(gain) != kUsGain) {kUsGain = gain;}
    }
  }

  /**
   * Group number
   */
  {
    // rclcpp don't support uint_16t as parameter
    std::unordered_set<int> groupNumber;
    int imuGroup = static_cast<int>(kImuSensorGroupNo);
    int usGroup = static_cast<int>(kUSSensorGroupNo);
    int magGroup = static_cast<int>(kMagSensorGroupNo);
    int hdGroup = static_cast<int>(kHallDtSensorGroupNo);
    int hd8Group = static_cast<int>(kHallDt8SensorGroupNo);
    int encStepGroup = static_cast<int>(kEncStepSensorGroupNo);
    int encCmGroup = static_cast<int>(kEncCmSensorGroupNo);
    int pbGroup = static_cast<int>(kButtonsSensorGroupNo);
    int vbatGroup = static_cast<int>(kBatteryVoltageSensorGroupNo);

    imuGroup = getValueFromNode(node, "sensor_groups.imu.group_num", imuGroup);
    usGroup = getValueFromNode(node, "sensor_groups.us.group_num", usGroup);
    magGroup = getValueFromNode(node, "sensor_groups.mag.group_num", magGroup);
    hdGroup = getValueFromNode(node, "sensor_groups.hall_dt.group_num", hdGroup);
    hd8Group = getValueFromNode(node, "sensor_groups.hall_dt8.group_num", hd8Group);
    encStepGroup = getValueFromNode(node, "sensor_groups.enc_step.group_num", encStepGroup);
    encCmGroup = getValueFromNode(node, "sensor_group.enc_cm.group_num", encCmGroup);
    pbGroup = getValueFromNode(node, "sensor_groups.pb.group_num", pbGroup);
    vbatGroup = getValueFromNode(node, "sensor_groups.vbat.group_num", vbatGroup);

    groupNumber.emplace(imuGroup);
    groupNumber.emplace(usGroup);
    groupNumber.emplace(magGroup);
    groupNumber.emplace(hdGroup);
    groupNumber.emplace(hd8Group);
    groupNumber.emplace(encStepGroup);
    groupNumber.emplace(encCmGroup);
    groupNumber.emplace(pbGroup);
    groupNumber.emplace(vbatGroup);

    if (groupNumber.size() != 9) {
      RCLCPP_INFO(node.get_logger(), "Group Number can not be the same! Using default!");
    } else {
      if (imuGroup >= 0 && imuGroup <= 19) {
        if (static_cast<uint16_t>(imuGroup) != kImuSensorGroupNo) {
          kImuSensorGroupNo = static_cast<uint16_t>(imuGroup);
        }
      } else {
        RCLCPP_INFO(
          node.get_logger(), "sensor_groups.imu.group_num out of range(0~19)! Using default!");
      }

      if (usGroup >= 0 && usGroup <= 19) {
        if (static_cast<uint16_t>(usGroup) != kUSSensorGroupNo) {
          kUSSensorGroupNo = static_cast<uint16_t>(usGroup);
        }
      } else {
        RCLCPP_INFO(
          node.get_logger(), "sensor_groups.us.group_num out of range(0~19)! Using default!");
      }

      if (magGroup >= 0 && magGroup <= 19) {
        if (static_cast<uint16_t>(magGroup) != kMagSensorGroupNo) {
          kMagSensorGroupNo = static_cast<uint16_t>(magGroup);
        }
      } else {
        RCLCPP_INFO(
          node.get_logger(), "sensor_groups.mag.group_num out of range(0~19)! Using default!");
      }

      if (hdGroup >= 0 && hdGroup <= 19) {
        if (static_cast<uint16_t>(hdGroup) != kHallDtSensorGroupNo) {
          kHallDtSensorGroupNo = static_cast<uint16_t>(hdGroup);
        }
      } else {
        RCLCPP_INFO(
          node.get_logger(), "sensor_groups.hall_dt.group_num out of range(0~19)! Using default!");
      }
      if (hd8Group >= 0 && hd8Group <= 19) {
        if (static_cast<uint16_t>(hd8Group) != kHallDt8SensorGroupNo) {
          kHallDt8SensorGroupNo = static_cast<uint16_t>(hd8Group);
        }
      } else {
        RCLCPP_INFO(
          node.get_logger(), "sensor_groups.hall_dt8.group_num out of range(0~19)! Using default!");
      }
      if (encStepGroup >= 0 && encStepGroup <= 19) {
        if (static_cast<uint16_t>(encStepGroup) != kEncStepSensorGroupNo) {
          kEncStepSensorGroupNo = static_cast<uint16_t>(encStepGroup);
        }
      } else {
        RCLCPP_INFO(
          node.get_logger(), "sensor_groups.enc_step.group_num out of range(0~19)! Using default!");
      }
      if (encCmGroup >= 0 && encCmGroup <= 19) {
        if (static_cast<uint16_t>(encCmGroup) != kEncCmSensorGroupNo) {
          kEncCmSensorGroupNo = static_cast<uint16_t>(encCmGroup);
        }
      } else {
        RCLCPP_INFO(
          node.get_logger(), "sensor_groups.enc_cm.group_num out of range(0~19)! Using default!");
      }
      if (vbatGroup >= 0 && vbatGroup <= 19) {
        if (static_cast<uint16_t>(vbatGroup) != kBatteryVoltageSensorGroupNo) {
          kBatteryVoltageSensorGroupNo = static_cast<uint16_t>(vbatGroup);
        }
      } else {
        RCLCPP_INFO(
          node.get_logger(), "sensor_groups.vbat.group_num out of range(0~19)! Using default!");
      }
      if (pbGroup >= 0 && pbGroup <= 19) {
        if (static_cast<uint16_t>(pbGroup) != kButtonsSensorGroupNo) {
          kButtonsSensorGroupNo = static_cast<uint16_t>(pbGroup);
        }
      } else {
        RCLCPP_INFO(
          node.get_logger(), "sensor_groups.pb.group_num out of range(0~19)! Using default!");
      }
    }
  }
  /**
   * Topic Name
   */
  {
    kTopicRc = getValueFromNode(node, "topic_name.rc", kTopicRc);
    kTopicSpeed = getValueFromNode(node, "topic_name.get_speed", kTopicSpeed);
    kTopicSteer = getValueFromNode(node, "topic_name.get_steer", kTopicSteer);
    kTopicButton = getValueFromNode(node, "topic_name.get_button", kTopicButton);
    kTopicDtData = getValueFromNode(node, "topic_name.dt", kTopicDtData);
    kTopicDt8Data = getValueFromNode(node, "topic_name.dt8", kTopicDt8Data);
    kTopicHallCntData = getValueFromNode(node, "topic_name.hall_cnt", kTopicHallCntData);
    kTopicImuData = getValueFromNode(node, "topic_name.imu", kTopicImuData);
    kTopicMagData = getValueFromNode(node, "topic_name.mag", kTopicMagData);
    kTopicPbData = getValueFromNode(node, "topic_name.pb", kTopicPbData);
    kTopicVbatData = getValueFromNode(node, "topic_name.vbat", kTopicVbatData);
    kTopicUsRangeSuffix = getValueFromNode(node, "topic_name.us_range_suffix", kTopicUsRangeSuffix);
    kTopicSetMotorLevelForward = getValueFromNode(
      node, "topic_name.motor_forward",
      kTopicSetMotorLevelForward);
    kTopicSetMotorLevelBackward = getValueFromNode(
      node, "topic_name.motor_backward",
      kTopicSetMotorLevelBackward);
    kTopicSetLed = getValueFromNode(node, "topic_name.led", kTopicSetLed);
    kTopicEncStepData = getValueFromNode(node, "topic_name.enc_step", kTopicEncStepData);
    kTopicEncCmData = getValueFromNode(node, "topic_name.enc_cm", kTopicEncCmData);
    kTopicSetSteeringAngle = getValueFromNode(
      node, "topic_name.set_steering_angle",
      kTopicSetSteeringAngle);
    kTopicTxtDisplay = getValueFromNode(node, "topic_name.display", kTopicTxtDisplay);
    kTopicUnkGrp = getValueFromNode(node, "topic_name.unknown_grp", kTopicUnkGrp);
    kTopicSetUnkGrp = getValueFromNode(node, "topic_name.set_unknown_grp", kTopicSetUnkGrp);
  }
}

std::string & Configuration::serialPort()
{
  static std::string result;
  readValue("serialPort", result);
  return result;
}

void Configuration::initializeSerialPortConfiguration()
{
  insertValue("serialPort", DEFAULT_SERIAL_PORT);
}

void Configuration::initializeSensorGroupConfiguration()
{
  insertValue("activateImuSensorGroup", DEFAULT_ACTIVATE_IMU_SENSOR_GROUP);
  insertValue("activateUSSensorGroup", DEFAULT_ACTIVATE_US_SENSOR_GROUP);
  insertValue("activateMagSensorGroup", DEFAULT_ACTIVATE_MAG_SENSOR_GROUP);
  insertValue("activateHallDtSensorGroup", DEFAULT_ACTIVATE_HALL_DT_SENSOR_GROUP);
  insertValue("activateHallDt8SensorGroup", DEFAULT_ACTIVATE_HALL_DT8_SENSOR_GROUP);
  insertValue("activateEncStepSensorGroup", DEFAULT_ACTIVATE_ENC_STEP_SENSOR_GROUP);
  insertValue("activateEncCmSensorGroup", DEFAULT_ACTIVATE_ENC_CMS_SENSOR_GROUP);
  insertValue("activatePbSensorGroup", DEFAULT_ACTIVATE_PB_SENSOR_GROUP);
  insertValue("activateVbatSensorGroup", DEFAULT_ACTIVATE_VBAT_SENSOR_GROUP);
  insertValue("activateTxtDisplay", DEFAULT_ACTIVATE_TXT_DISPLAY);
  insertValue("activateUnkGrp", DEFAULT_ACTIVATE_UNK_GRP);
  insertValue("activateRc", DEFAULT_ACTIVATE_RC);
  insertValue("imuTs", DEFAULT_IMU_TS);
  insertValue("encStepSkip", DEFAULT_ENC_STEP_SKIP);
  insertValue("encCmSkip", DEFAULT_ENC_CM_SKIP);
  insertValue("rcFrequency", DEFAULT_RC_FREQUENCY);
  insertValue("steerFrequency", DEFAULT_STEER_FREQUENCY);
}

bool Configuration::activateTxtDisplay()
{
  bool result;
  readValue("activateTxtDisplay", result);
  return result;
}

bool Configuration::activateImuSensorGroup()
{
  bool result;
  readValue("activateImuSensorGroup", result);
  return result;
}

bool Configuration::activateUSSensorGroup()
{
  bool result;
  readValue("activateUSSensorGroup", result);
  return result;
}

bool Configuration::activateMagSensorGroup()
{
  bool result;
  readValue("activateMagSensorGroup", result);
  return result;
}

bool Configuration::activateHallDtSensorGroup()
{
  bool result;
  readValue("activateHallDtSensorGroup", result);
  return result;
}

bool Configuration::activateHallDt8SensorGroup()
{
  bool result;
  readValue("activateHallDt8SensorGroup", result);
  return result;
}

bool Configuration::activateEncStepSensorGroup()
{
  bool result;
  readValue("activateEncStepSensorGroup", result);
  return result;
}

bool Configuration::activateEncCmSensorGroup()
{
  bool result;
  readValue("activateEncCmSensorGroup", result);
  return result;
}

bool Configuration::activateRawCommunication()
{
  bool result;
  readValue("activateRawCommunication", result);
  return result;
}

bool Configuration::activatePbSensorGroup()
{
  bool result;
  readValue("activatePbSensorGroup", result);
  return result;
}

bool Configuration::activateVbatSensorGroup()
{
  bool result;
  readValue("activateVbatSensorGroup", result);
  return result;
}

bool Configuration::activateUnkGrp()
{
  bool result;
  readValue("activateUnkGrp", result);
  return result;
}

bool Configuration::activateRc()
{
  bool result;
  readValue("activateRc", result);
  return result;
}

void Configuration::initializeGeneralConfiguration()
{
  insertValue("activateRawCommunication", DEFAULT_ACTIVATE_RAW_COMMUNICATION);
  logLevelMapping = {
    {"fatal", RCUTILS_LOG_SEVERITY_FATAL},
    {"error", RCUTILS_LOG_SEVERITY_ERROR},
    {"warn", RCUTILS_LOG_SEVERITY_WARN},
    {"info", RCUTILS_LOG_SEVERITY_INFO},
    {"debug", RCUTILS_LOG_SEVERITY_DEBUG}
  };

  imuAccMapping = {
    {2, CommandElement::IMUAcceleration::ImuAcc2},
    {4, CommandElement::IMUAcceleration::ImuAcc4},
    {8, CommandElement::IMUAcceleration::ImuAcc8},
    {16, CommandElement::IMUAcceleration::ImuAcc16}
  };

  imuRotMapping = {
    {250, CommandElement::IMURotationRate::ImuRot250},
    {500, CommandElement::IMURotationRate::ImuRot500},
    {1000, CommandElement::IMURotationRate::ImuRot1000},
    {2000, CommandElement::IMURotationRate::ImuRot2000}
  };

  imuAfiltMapping = {
    {-1, CommandElement::IMUAFiltMode::AFILT_1},
    {0, CommandElement::IMUAFiltMode::AFILT0},
    {1, CommandElement::IMUAFiltMode::AFILT1},
    {2, CommandElement::IMUAFiltMode::AFILT2},
    {3, CommandElement::IMUAFiltMode::AFILT3},
    {4, CommandElement::IMUAFiltMode::AFILT4},
    {5, CommandElement::IMUAFiltMode::AFILT5},
    {6, CommandElement::IMUAFiltMode::AFILT6},
    {7, CommandElement::IMUAFiltMode::AFILT7}
  };

  imuGfiltMapping = {
    {-2, CommandElement::IMUGFiltMode::GFILT_2},
    {-1, CommandElement::IMUGFiltMode::GFILT_1},
    {0, CommandElement::IMUGFiltMode::GFILT0},
    {1, CommandElement::IMUGFiltMode::GFILT1},
    {2, CommandElement::IMUGFiltMode::GFILT2},
    {3, CommandElement::IMUGFiltMode::GFILT3},
    {4, CommandElement::IMUGFiltMode::GFILT4},
    {5, CommandElement::IMUGFiltMode::GFILT5},
    {6, CommandElement::IMUGFiltMode::GFILT6},
    {7, CommandElement::IMUGFiltMode::GFILT7},
  };

  insertValue("logLevel", logLevelMapping[DEFAULT_LOG_LEVEL]);
}

int Configuration::encStepSkip()
{
  int result = 0;
  readValue("encStepSkip", result);
  return result;
}

int Configuration::encCmSkip()
{
  int result = 0;
  readValue("encCmSkip", result);
  return result;
}

int Configuration::deadMan()
{
  int result = 0;
  readValue("deadMan", result);
  return result;
}

int Configuration::imuTs()
{
  int result = 0;
  readValue("imuTs", result);
  return result;
}

int Configuration::board_version()
{
  int result = 0;
  readValue<int>("boardVersion", result);
  return result;
}

bool Configuration::reset_start()
{
  bool result = true;
  readValue<bool>("reset_start", result);
  return result;
}

bool Configuration::reset_end()
{
  bool result = true;
  readValue<bool>("reset_end", result);
  return result;
}

int Configuration::rc_frequency()
{
  int result = 10;
  readValue<int>("rcFrequency", result);
  return result;
}

int Configuration::steer_frequency()
{
  int result = 10;
  readValue<int>("steerFrequency", result);
  return result;
}

int Configuration::logLevel()
{
  int result = 0;
  readValue<int>("logLevel", result);
  return result;
}
