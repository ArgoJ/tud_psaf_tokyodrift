#ifndef PSAF_UCBRIDGE__DEFAULT_CONFIGURATION_HPP_
#define PSAF_UCBRIDGE__DEFAULT_CONFIGURATION_HPP_

/**
 * Define default values as macros
 */
#define STRING(x) std::string(x)

// general
#define DEFAULT_ACTIVATE_RAW_COMMUNICATION false
#define DEFAULT_TOPIC_RAW_COMMUNICATION "send_command_raw"
#define DEFAULT_LOG_LEVEL STRING("info")

// serial port configuration
#define DEFAULT_SERIAL_PORT STRING("/dev/ttyUSB0")

// sensor group configuration
#define DEFAULT_ACTIVATE_IMU_SENSOR_GROUP true
#define DEFAULT_ACTIVATE_US_SENSOR_GROUP true
#define DEFAULT_ACTIVATE_MAG_SENSOR_GROUP true
#define DEFAULT_ACTIVATE_HALL_DT_SENSOR_GROUP true
#define DEFAULT_ACTIVATE_HALL_DT8_SENSOR_GROUP true
#define DEFAULT_ACTIVATE_ENC_STEP_SENSOR_GROUP true
#define DEFAULT_ACTIVATE_ENC_CMS_SENSOR_GROUP true
#define DEFAULT_ACTIVATE_PB_SENSOR_GROUP true
#define DEFAULT_ACTIVATE_VBAT_SENSOR_GROUP true
#define DEFAULT_ACTIVATE_TXT_DISPLAY false
#define DEFAULT_ACTIVATE_UNK_GRP false
#define DEFAULT_ACTIVATE_RC true
#define DEFAULT_RC_FREQUENCY 100
#define DEFAULT_STEER_FREQUENCY 100
#define DEFAULT_BOARD_VERSION 2
#define DEFAULT_RESET_START true
#define DEFAULT_RESET_END false
#define DEFAULT_IMU_TS 10
#define DEFAULT_ENC_STEP_SKIP 5
#define DEFAULT_ENC_CM_SKIP 5
#define DEFAULT_DEAD_MAN 200

#endif  // PSAF_UCBRIDGE__DEFAULT_CONFIGURATION_HPP_
