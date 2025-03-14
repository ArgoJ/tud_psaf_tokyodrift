#ifndef PSAF_UCBRIDGE__DYNAMIC_CONFIG_HPP_
#define PSAF_UCBRIDGE__DYNAMIC_CONFIG_HPP_
#include <string>
#include <map>
#include "psaf_ucbridge/configuration/static_config.hpp"
class DynamicConfig
{
public:
  /**
   * Default config for IMU.
   */
  CommandElement::IMUAcceleration kImuAcceleration = CommandElement::IMUAcceleration::ImuAcc4;
  CommandElement::IMURotationRate kImuRotationRate = CommandElement::IMURotationRate::ImuRot500;
  CommandElement::IMUAFiltMode kImuAFilt = CommandElement::IMUAFiltMode::AFILT0;
  CommandElement::IMUGFiltMode kImuGFilt = CommandElement::IMUGFiltMode::GFILT0;

  /**
   * Default config for MAG
   */
  bool kMagUseasa = true;

  /**
   * Default config for US.
   */
  uint32_t kUsRange = 100;
  uint32_t kUsGain = 10;

  /**
   * Default config for Sensor Groups.
   */
  uint16_t kImuSensorGroupNo = 0;

  uint16_t kUSSensorGroupNo = 1;

  uint16_t kMagSensorGroupNo = 2;

  uint16_t kHallDtSensorGroupNo = 3;

  uint16_t kHallDt8SensorGroupNo = 4;

  uint16_t kEncStepSensorGroupNo = 5;

  uint16_t kEncCmSensorGroupNo = 6;

  uint16_t kBatteryVoltageSensorGroupNo = 7;

  uint16_t kButtonsSensorGroupNo = 8;

  std::string kTopicSpeed = "uc_bridge/get_speed";

  std::string kTopicSteer = "uc_bridge/get_steering";

  std::string kTopicButton = "uc_bridge/button";

  /**
   * On this topic unknown group is published
   */
  std::string kTopicUnkGrp = "unknown_grp";

  /**
   * On this topic unknown group will be set
   */
  std::string kTopicSetUnkGrp = "set_unknown_grp";

  /**
   * On this topic dt is published
   */
  std::string kTopicDtData = "dt_data";

  /**
* On this topic dt8 is published
*/
  std::string kTopicDt8Data = "dt8_data";

  /**
   * On this topic enc step is published
   */
  std::string kTopicEncStepData = "enc_step_data";

  /**
   * On this topic enc cm is published
   */
  std::string kTopicEncCmData = "enc_cm_data";

  /**
   * On this topic rc is published
   */
  std::string kTopicRc = "uc_bridge/manual_signals";

  /**
   * On this topic PB data are published
   */
  std::string kTopicPbData = "pb_data";

  /**
   * On this topic vbat data are published
   */
  std::string kTopicVbatData = "vbat_data";

  /**
 * On this topic the number of hall sensor impulses are published
 */
  std::string kTopicHallCntData = "hall_cnt_data";

  /**
 * On this topic all imu data is published.
 */
  std::string kTopicImuData = "imu_data";

  /**
 * On this topic all data from the magnetometer is published.
 */
  std::string kTopicMagData = "mag_data";

  std::map<std::string, std::string> kTopicRange{
    {"US1", "uc_bridge/us_rear_left"},
    {"US2", "uc_bridge/us_mid_right"},
    {"US3", "uc_bridge/us_front_right"},
    {"US4", "uc_bridge/us_front_center"},
    {"US5", "uc_bridge/us_mid_left"},
    {"US6", "uc_bridge/us_rear_right"},
    {"US7", "uc_bridge/us_rear_center"},
    {"US8", "uc_bridge/us_front_left"},
    {"USF", "uc_bridge/us_front_center"},
    {"USL", "uc_bridge/us_mid_left"},
    {"USR", "uc_bridge/us_mid_right"},
    {"USB", "uc_bridge/us_rear_center"}
  };

  std::string kTopicUsRangeSuffix = "_range";

  std::string kTopicSetMotorLevelForward = "uc_bridge/set_motor_level_forward";

  std::string kTopicSetMotorLevelBackward = "uc_bridge/set_motor_level_backward";

  std::string kTopicSetSteeringAngle = "uc_bridge/set_steering";

  std::string kTopicSetLed = "uc_bridge/light";

  std::string kTopicTxtDisplay = "display";


  /**
   * On these services name are defined here.
   */
  std::string kServiceReqVer = "version_request";
  std::string kServiceReqId = "ID_request";
  std::string kServiceReqSid = "SID_request";
  std::string kServiceReqSteer = "Steer_request";
  std::string kServiceReqDrv = "Drv_request";
  std::string kServiceReqDms = "Dms_request";
  std::string kServiceReqChs = "Chs_request";
  std::string kServiceReqGet = "Get_request";
  std::string kServiceReqGrp = "Grp_request";
  std::string kServiceReqVout = "Vout_request";
  std::string kServiceReqUs = "Us_request";
  std::string kServiceReqImu = "Imu_request";
  std::string kServiceReqMag = "Mag_request";
  std::string kServiceReqEnc = "Enc_request";
  std::string kServiceReqRaw = "raw_communication";
};
#endif  // PSAF_UCBRIDGE__DYNAMIC_CONFIG_HPP_
