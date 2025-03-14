//
// Created by huwang on 06.06.21.
//

#ifndef PSAF_UCBRIDGE__STATIC_CONFIG_HPP_
#define PSAF_UCBRIDGE__STATIC_CONFIG_HPP_
#include <charconv>
#include <string>
#include <array>
#include <vector>
#include <map>
#include <cmath>
#include <iostream>
#include <unordered_map>
/**
 * The physic constant value can not be changed.
 * @note using by object reference(CommandElement).
 */
class PhysicValue
{
public:
  constexpr static const double kSpeedOfSound = 343.2;
  constexpr static const double kGravity = 9.81;
  constexpr static const double kDegreeToRad = M_PI / 360.0;
  constexpr static const int kMinSteerValue = -1000;
  constexpr static const int kMaxSteerValue = 1000;
  constexpr static const int kMinDrivingSpeed = -500;
  constexpr static const int kMaxDrivingSpeed = 1000;
  constexpr static const int kMinDrivingSpeedBackwards = 0;
  constexpr static const int kMaxDrivingSpeedBackwards = 500;
  constexpr static float kAxConversionFactor = 0.00119750976;
  constexpr static float kAyConversionFactor = 0.00119750976;
  constexpr static float kAzConversionFactor = 0.00119750976;
  static constexpr float kWxConversionFactor = 0.00026646248;
  static constexpr float kWyConversionFactor = 0.00026646248;
  static constexpr float kWzConversionFactor = 0.00026646248;
  static constexpr float kUltrasonicConversionFactor = 0.0001;
  static constexpr float kMxConversionFactor = 0.15;
  static constexpr float kMyConversionFactor = 0.15;
  static constexpr float kMzConversionFactor = 0.15;
  static constexpr float kVolatgeConversionFactor = 1.0;
  static constexpr float kDt8ConversionFactor = 0.001;
  static constexpr float kDtConversionFactor = 0.0001;
};

/**
 * General command element of the board.
 */
class GeneralCommandElement
{
public:
  enum MessageType {WAIT, DISCARD, ASYNC};
  /**
 * Prefix which is sent before sensor group data
 */
  const char kChannelGroupMessagePrefix = '#';

  /**
   * Prefix which is sent before text data
   */
  const char kTextMessagePrefix = '\'';

  /**
   * Prefix which is sent before answer to request
   */
  const char kAnswerOnCommandPrefix = ':';
  /**
   * The last character of every message.
   */
  const char kMessageEndChar = '\n';

  /**
   * The first character of all commands.
   */
  const char kCommandStartChar = '!';

  /**
   * The first character of all requests.
   */
  const char kRequestStartChar = '?';

  /**
   * The main message of all reset messages.
   */
  const std::string kMainMessageReset = "RESET NOW";

  /**
   * The main message of all LED messages.
   */
  const std::string kMainMessageLed = "LED";

  /**
   * The main message of all Session ID messages.
   */
  const std::string kMainMessageSid = "SID";

  /**
   * The main message of all steer messages.
   */
  const std::string kMainMessageSteer = "STEER";

  /**
   * The main message of all driving messages.
   */
  const std::string kMainMessageDrv = "DRV";

  /**
   * The main message of all data acquisition messages.
   */
  const std::string kMainMessageDaq = "DAQ";

  /**
   * The main message of all enc message.
   */
  const std::string kMainMessageEnc = "ENC";

  /**
   * The main message of remote control indicator
   */
  const std::string kMianMessageRc = "RCMODE";


  /**
   * The main message of all vout Messages.
   */
  const std::string kMainMessageVout = "VOUT";

  /**
   * The main message of all IMU messages.
   */
  const std::string kMainMessageImu = "IMU";

  /**
   * The main message of all magnetometer messages.
   */
  const std::string kMainMessageMag = "MAG";

  /**
   * The main message of all ultrasonic sensor messages.
   */
  const std::string kMainMessageUs = "US";

  /**
   * The main message of all version messages.
   */
  const std::string kMainMessageVer = "VER";

  /**
   * The main message of all ID messages.
   */
  const std::string kMainMessageId = "ID";

  /**
   * The main message of all tics messages.
   */
  const std::string kMainMessageTics = "TICS";


  const std::string kOn = " ON";
  const std::string kOff = " OFF";

  /**
   * The first character of every response from the ucboard.
   */
  const char kResponseStartChar = '\u0002';

  /**
   * The last character of every response from the ucboard.
   */
  const char kResponseEndChar = '\u0003';
};

/**
 * LED specified command elements and enums.
 */
class LEDCommandElement
{
public:
  enum LEDType {A, B, BRMS, BLKL, BLKR};
  enum LEDMode {ON, OFF, TOGGLE, BLINK, NONE};
  struct LEDOptions
  {
    LEDMode mode = LEDMode::NONE;
    bool inverted = false;
    std::vector<uint32_t> on_times;
    std::vector<uint32_t> off_times;
  };

  const std::string kLedInverted = " 0";
  inline static const std::map<LEDCommandElement::LEDType, std::string> type2string{
    {LEDType::A, " A"},
    {LEDType::B, " B"},
    {LEDType::BRMS, " BRMS"},
    {LEDType::BLKL, " BLKL"},
    {LEDType::BLKR, " BLKR"}
  };
  inline static const std::map<LEDMode, std::string> mode2string{
    {ON, " ON"},
    {OFF, " OFF"},
    {TOGGLE, " TOG"},
  };
};

/**
 * DRV specified command elements and enums.
 */
class DRVCommendElement
{
public:
  enum DRVMode {FORWARDS,
    BACKWARDS,
    DIRECT,
    OFF,
    ON,
    DMS,
    NIL};
  const std::string kDrvF = " F";
  const std::string kDrvB = " B";
  const std::string kDrvD = " D";
  const std::string kDrvOn = " ON";
  const std::string kDrvOff = " OFF";
  // maybe support without parameter?
  const std::string kDrvDms = " ~DMS";
};

/**
 * DRV specified command elements and enums.
 */
class DAQCommandElement
{
public:
  enum DAQEncoding {ASCII, HEX, BASE64};
  enum DAQGroupMode
  {
    CREATE,
    DELETE,
    ACTIVATE,
    DEACTIVATE
  };
  enum DAQMode
  {
    NIL,
    START,
    STOP,
    GROUP
  };
  enum DAQScanningMode
  {
    ALL,
    ANY,
    TS
  };

  const std::string kDaqGrp = " GRP ";  // Group string in the command.
  const std::string kDaqGrps = " GRPS";
  const std::string kDaqDelete = " ~DELETE";  // Delete a group.
  const std::string kDaqActivate = " ~ACTIVATE";  // Activate a group.
  const std::string kDaqDeactivate = " ~DEACTIVATE";  // Deactivate a group.
  const std::string kDaqStart = " START";  // Start the data acquisition.
  const std::string kDaqStop = " STOP";  // Stop the data acquisition.

  const std::string kDaqAll = " ~ALL";
  const std::string kDaqAny = " ~ANY";
  const std::string kDaqTs = " ~TS";
  const std::string kDaqAvg = " ~AVG";

  const std::string kDaqSkip = " ~SKIP";
  const std::string kDaqEnc = " ~ENC";
  const std::string kDaqCrc = " ~CRC";
  const std::string kDaqAge = " ~AGE";
  const std::string kDaqTics = " ~TICS";
  const std::string kDaqB64 = "B64";
  const std::string kDaqHex = "HEX";
  const std::string kDaqAscii = "ASCII";

  static constexpr int kDaqMinGroupNumber = 0;

  static constexpr int kDaqMaxGroupNumber = 19;
};

/**
 * DAQ channel maps.
 */
class DAQChannelElement
{
public:
  enum ChannelEnum
  {
    AX,
    AY,
    AZ,
    GX,
    GY,
    GZ,
    MX,
    MY,
    MZ,
    USF,
    USL,
    USR,
    USB,
    US1,
    US2,
    US3,
    US4,
    US5,
    US6,
    US7,
    US8,
    VSBAT,
    VDBAT,
    HALL_DT,
    HALL_DT8,
    HALL_CNT,
    ENC_STEPS,
    ENC_CMS,
    PBA,
    PBB,
    PBC,
    TICS,
    UNK0,
    UNK1,
    UNK2,
    UNK3,
    UNK4,
    UNK5
  };

  const std::map<ChannelEnum, std::string> channelNames = {
    {ChannelEnum::AX, "AX"},
    {ChannelEnum::AY, "AY"},
    {ChannelEnum::AZ, "AZ"},
    {ChannelEnum::GX, "GX"},
    {ChannelEnum::GY, "GY"},
    {ChannelEnum::GZ, "GZ"},
    {ChannelEnum::MX, "MX"},
    {ChannelEnum::MY, "MY"},
    {ChannelEnum::MZ, "MZ"},
    {ChannelEnum::US1, "US1"},
    {ChannelEnum::US2, "US2"},
    {ChannelEnum::US3, "US3"},
    {ChannelEnum::US4, "US4"},
    {ChannelEnum::US5, "US5"},
    {ChannelEnum::US6, "US6"},
    {ChannelEnum::US7, "US7"},
    {ChannelEnum::US8, "US8"},
    {ChannelEnum::USF, "USF"},
    {ChannelEnum::USL, "USL"},
    {ChannelEnum::USR, "USR"},
    {ChannelEnum::USB, "USB"},
    {ChannelEnum::VSBAT, "VSBAT"},
    {ChannelEnum::VDBAT, "VDBAT"},
    {ChannelEnum::HALL_DT, "HALL_DT"},
    {ChannelEnum::HALL_DT8, "HALL_DT8"},
    {ChannelEnum::HALL_CNT, "HALL_CNT"},
    {ChannelEnum::PBA, "PBA"},
    {ChannelEnum::PBB, "PBB"},
    {ChannelEnum::PBC, "PBC"},
    {ChannelEnum::ENC_CMS, "ENC_CMS"},
    {ChannelEnum::ENC_STEPS, "ENC_STEPS"},
    {ChannelEnum::TICS, "TICS"},
    {ChannelEnum::UNK0, "UNK0"},
    {ChannelEnum::UNK1, "UNK1"},
    {ChannelEnum::UNK2, "UNK2"},
    {ChannelEnum::UNK3, "UNK3"},
    {ChannelEnum::UNK4, "UNK4"},
    {ChannelEnum::UNK5, "UNK5"}
  };

  const std::unordered_map<std::string, ChannelEnum> kDaqUsChannels = {
    {"US1", US1},
    {"US2", US2},
    {"US3", US3},
    {"US4", US4},
    {"US5", US5},
    {"US6", US6},
    {"US7", US7},
    {"US8", US8},
    {"USF", USF},
    {"USL", USL},
    {"USR", USR},
    {"USB", USB}
  };
};

/**
 * IMU specified command elements and enums.
 */
class IMUCommandElement
{
public:
  enum IMUAcceleration { ImuAcc2 = 2, ImuAcc4 = 4, ImuAcc8 = 8, ImuAcc16 = 16 };
  const std::map<IMUAcceleration, double> ImuAccelerationConversion = {
    {ImuAcc2, PhysicValue::kGravity / 16384.0},
    {ImuAcc4, PhysicValue::kGravity / 8192.0},
    {ImuAcc8, PhysicValue::kGravity / 4096.0},
    {ImuAcc16, PhysicValue::kGravity / 2048.0}
  };
  enum IMUAFiltMode
  {
    AFILT_1 = -1,
    AFILT0 = 0,
    AFILT1 = 1,
    AFILT2 = 2,
    AFILT3 = 3,
    AFILT4 = 4,
    AFILT5 = 5,
    AFILT6 = 6,
    AFILT7 = 7
  };
  enum IMUGFiltMode
  {
    GFILT_2 = -2,
    GFILT_1 = -1,
    GFILT0 = 0,
    GFILT1 = 1,
    GFILT2 = 2,
    GFILT3 = 3,
    GFILT4 = 4,
    GFILT5 = 5,
    GFILT6 = 6,
    GFILT7 = 7
  };
  enum IMURotationRate { ImuRot250 = 250, ImuRot500 = 500, ImuRot1000 = 1000, ImuRot2000 = 2000 };
  const std::map<IMURotationRate, double> ImuRotationRateConversion = {
    {ImuRot250, PhysicValue::kDegreeToRad / 131.0},
    {ImuRot500, PhysicValue::kDegreeToRad / 65.5},
    {ImuRot1000, PhysicValue::kDegreeToRad / 32.8},
    {ImuRot2000, PhysicValue::kDegreeToRad / 16.4}
  };

  const std::string kImuOpt = " OPT";
  const std::string kImuArange = " ~ARANGE";
  const std::string kImuAfilt = " ~AFILT";
  const std::string kImuGrange = " ~GRANGE";
  const std::string kImuGfilt = " ~GFILT";
};

/**
 * MAG specified command elements and enums.
 */
class MAGCommandElement
{
public:
  const std::string kMagOpt = " OPT";
  const std::string kMagAsa = " ~USEASA";
  const std::string kMagAsaq = " ASA";
};

/**
 * US specified command elements and enums.
 */
class USCommandElement
{
  // TODO(Jingcun) Ping and CHGADDR are not implemented!

public:
  enum USMode {ON, OFF, OPT};
  const std::string kUsOn = " ON";
  const std::string kUsOff = " OFF";
  const std::string kUsOpt = " OPT";
  const std::string kUsGain = " ~GAIN";
  const std::string kUsRange = " ~RANGE";
};

/**
 * Using by object reference. The interface of all the static command elements
 * and physic value.
 */
class CommandElement
  : public GeneralCommandElement,
  public PhysicValue,
  public DAQChannelElement,
  public DAQCommandElement,
  public LEDCommandElement,
  public DRVCommendElement,
  public IMUCommandElement,
  public MAGCommandElement,
  public USCommandElement
{
public:
  ~CommandElement() = default;
  CommandElement(const CommandElement &) = delete;
  CommandElement & operator=(const CommandElement &) = delete;
  static CommandElement & command_element() {static CommandElement _instance; return _instance;}

private:
  CommandElement() = default;
};
#endif  // PSAF_UCBRIDGE__STATIC_CONFIG_HPP_
