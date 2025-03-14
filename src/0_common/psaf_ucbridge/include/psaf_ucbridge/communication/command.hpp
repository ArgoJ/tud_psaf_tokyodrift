#ifndef COMMAND_HPP
#define COMMAND_HPP

#include <charconv>
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include "psaf_ucbridge/communication/message.hpp"
#include "psaf_ucbridge/configuration/static_config.hpp"
#include "psaf_ucbridge/configuration/configuration.hpp"

/**
 * Command extends the Message Class by setting a startchar.
 * Command is a special form of a message.
 * @brief more specified Message
 */
class Command : public Message
{
public:
  /**
   * Constructor sets startchar to the char set in the configuration which is the same for all
   * command-messages.
   */
  explicit Command(const std::string & main_message, bool requiresAnswer = true)
  : Message(CommandElement::command_element().kCommandStartChar,
      main_message, requiresAnswer) {}
};

/**
 * Raw communication with the Board.
 * @brief raw communication.
 */
class ComRaw : public Command
{
public:
  ComRaw();

  [[nodiscard]] std::string ReturnMessage() const final;

  void parseAnswer(std::string_view message) final;

  ComRaw & raw(const std::string & raw_message);

  ComRaw & answer(const std::string & answer);

  ComRaw & requireAnswer(
    bool requires
  );

private:
  std::string raw_message_;
  std::string answer_;
};

/**
 * Reset command with main message "reset now"
 * @brief Command Reset
 */
class ComReset : public Command
{
public:
  /**
   * Constructor sets main_message to "reset now"
   */
  ComReset();

  [[nodiscard]] std::string ReturnMessage() const final;

  void parseAnswer(std::string_view message) final;
};

/**
 * LED Command with main message "led"
 * @brief Command LED
 */
class ComLED : public Command
{
public:
  /**
   * Constructor sets main_message to "led"
   */
  ComLED();

  ComLED & on(CommandElement::LEDType type);

  ComLED & off(CommandElement::LEDType type);

  ComLED & toggle(CommandElement::LEDType type);

  ComLED & blink(CommandElement::LEDType type, uint32_t on_time, uint32_t off_time);

  ComLED & invert(CommandElement::LEDType type, bool inverted = true);

  [[nodiscard]] std::string ReturnMessage() const final;

  void parseAnswer(std::string_view message) final;

private:
  std::map<CommandElement::LEDType, CommandElement::LEDOptions> all_leds_;
//  CommandElement::LEDOptions optA, optB;
  static void addBlinkSequence(const CommandElement::LEDOptions & options, std::stringstream & ss);

  void setMode(CommandElement::LEDType type, CommandElement::LEDMode mode);
  void addSequence(
    CommandElement::CommandElement::LEDType type,
    uint32_t on_time,
    uint32_t off_time);
};

/**
 * SID Command (Session ID) with main message "sid"
 * @brief Command Session ID
 */
class ComSID : public Command
{
public:
  /**
   * Constructor sets main_message to "sid"
   * @param to_be_checked_arg Optional argument which is needed for the command
   */
  ComSID();

  ComSID & sid(uint32_t sid);

  [[nodiscard]] std::string ReturnMessage() const final;

  void parseAnswer(std::string_view message) final;

private:
  uint32_t sid_ = 0;
};

/**
 * STEER Command (steering angle) with main message "steer"
 * @brief Command Steering angle
 */
class ComSTEER : public Command
{
public:
  /**
   * Constructor sets main_message to "steer"
   */
  ComSTEER();

  ComSTEER & steer(int angle);

  ComSTEER & repeat(bool repeat);

  [[nodiscard]] std::string ReturnMessage() const final;

  void parseAnswer(std::string_view message) final;

private:
  int angle_ = 0;
  bool repeat_ = false;
};

/**
 * DRV Command (Driving speed) with main message "drv"
 * @brief Command Driving speed
 */
class ComDRV : public Command
{
public:
  /**
   * Constructor sets main_message to "drv"
   */
  ComDRV();

  /**
   * Drive forwards
   * @param speed forwards driving speed
   */
  ComDRV & forwards(int speed);

  /**
  * Drive backwards
  * @param speed backwards driving speed
  */
  ComDRV & backwards(int speed);

  /**
   * Drive direct
   * @param speed direct driving speed
   */
  ComDRV & direct(int speed);

  /**
   * Deactivate driving controller
   * @return reference to this object
   */
  ComDRV & off();

  /**
   * Activate driving controller
   * @return reference to this object
   */
  ComDRV & on();

  /**
  * Activate dead man switch (DMS)
  * @return reference to this object
  */
  // data check?
  ComDRV & dms(int time);

  /**
  * Activate dead man switch (DMS)
  * @return reference to this object
  */
  ComDRV & repeat(bool repeat);

  [[nodiscard]] std::string ReturnMessage() const final;

  void parseAnswer(std::string_view message) final;

private:
  int speed_ = 0;
  int dms_time_ = 0;
  bool repeat_ = false;
  CommandElement::DRVMode mode_ = CommandElement::DRVMode::NIL;
};

/**
 * DAQ Command (Data Acquisition) with main message "daq"
 * @brief Command Data Aquisition
 */
class ComDAQ : public Command
{
public:
  /**
   * Constructor sets main_message to "daq"
   * @param to_be_checked_arg Optional argument which is needed for the command
   */

  ComDAQ();

  ComDAQ & group(int groupNo);

  ComDAQ & start();

  ComDAQ & stop();

  ComDAQ & remove();

  ComDAQ & deactivate();

  ComDAQ & activate();

  ComDAQ & channel(CommandElement::ChannelEnum c);

  ComDAQ & all(int maxwait = -1);

  ComDAQ & any();

  ComDAQ & ts(int ts);

  ComDAQ & avg(int ta = -1);

  ComDAQ & skip(int number);

  ComDAQ & encoding(CommandElement::DAQEncoding encoding);

  ComDAQ & crc();

  ComDAQ & age();

  ComDAQ & tics();

  ComDAQ & otherOpt(std::string opts);

  [[nodiscard]] std::string ReturnMessage() const final;

  [[nodiscard]] const std::vector<CommandElement::ChannelEnum> & getChannels() const;

  void parseAnswer(std::string_view message) final;

  [[nodiscard]] int groupNo() const;

  [[nodiscard]] CommandElement::DAQEncoding encoding() const;

private:
  int groupNo_ = -1;
  int ts_ = -1;
  int ta_ = -1;
  int maxwait_ = 10;
  bool avg_ = false;
  bool age_ = false;
  bool tics_ = false;
  int skip_ = -1;
  bool crc_ = false;
  std::string otherOpt_;
  CommandElement::DAQEncoding encoding_ = CommandElement::DAQEncoding::ASCII;
  CommandElement::DAQMode mode_ = CommandElement::DAQMode::NIL;
  CommandElement::DAQGroupMode groupMode_ = CommandElement::DAQGroupMode::CREATE;
  CommandElement::DAQScanningMode scanningMode_ = CommandElement::DAQScanningMode::ALL;
  std::vector<CommandElement::ChannelEnum> channels;
  static void checkGroupNo(int groupNo);
  void addScanningMode(std::stringstream & ss) const;
  void addGroupOptions(std::stringstream & ss) const;
  void addChannels(std::stringstream & ss) const;
  void addChannelOptions(std::stringstream & ss) const;
};

/**
 * VOUT Command (12V Output) with main message "vout"
 * @brief Command 12V Output
 */
class ComVOUT : public Command
{
public:
  /**
   * Constructor sets main_message to "vout"
   */
  ComVOUT();

  ComVOUT & on();

  ComVOUT & off();

  [[nodiscard]] std::string ReturnMessage() const final;

  void parseAnswer(std::string_view message) final;

private:
  bool on_ = true;
};

/**
 * IMU Command with main message "imu"
 * @brief Command IMU
 */
class ComIMU : public Command
{
public:
  /**
   * Constructor sets main_message to "imu"
   */
  ComIMU();

  ComIMU & arange(CommandElement::IMUAcceleration range);

  ComIMU & afilt(CommandElement::IMUAFiltMode mode);

  ComIMU & gfilt(CommandElement::IMUGFiltMode mode);

  ComIMU & grange(CommandElement::IMURotationRate range);

  [[nodiscard]] std::string ReturnMessage() const final;

  void parseAnswer(std::string_view message) final;

private:
  bool usingdefault = true;
  bool arange_ = false;
  bool grange_ = false;
  bool afilter_ = false;
  bool gfilter_ = false;

  CommandElement::IMUAcceleration arangeValue = Configuration::instance().kImuAcceleration;

  CommandElement::IMURotationRate grangeValue = Configuration::instance().kImuRotationRate;

  CommandElement::IMUAFiltMode afilterValue = Configuration::instance().kImuAFilt;

  CommandElement::IMUGFiltMode gfilterValue = Configuration::instance().kImuGFilt;
};

/**
 * MAG Command (Magnetometer) with main message "mag"
 * @brief Command Magnetometer
 */
class ComMAG : virtual public Command
{
public:
  /**
   * Constructor sets main_message to "mag"
   */
  ComMAG();

  ComMAG & useasa(bool activate = true);

  [[nodiscard]] std::string ReturnMessage() const final;

  void parseAnswer(std::string_view message) final;

private:
  bool useasa_ = Configuration::instance().kMagUseasa;
};

/**
 * US Command (Ultrasonic sensor) with main message "us"
 * @brief Command Ultrasonic sensor
 */
class ComUS : virtual public Command
{
public:
  /**
   * Constructor sets main_message to "us"
   * @param to_be_checked_arg Optional argument which is needed for the command
   */
  ComUS();

  ComUS & on();
  ComUS & off();
  ComUS & gain(uint32_t gain);
  ComUS & range(uint32_t range);

  [[nodiscard]] std::string ReturnMessage() const final;

  void parseAnswer(std::string_view message) final;

private:
  bool usingdefault = true;
  bool range_ = false;
  bool gain_ = false;
  CommandElement::USMode mode_ = CommandElement::USMode::OPT;
  uint32_t gainValue = Configuration::instance().kUsGain;
  uint32_t rangeValue = Configuration::instance().kUsRange;
};

/**
 * ENC Command (Incremental encoder) with main message "ENC"
 * @brief Command Incremental encoder
 */
class ComENC : virtual public Command
{
public:
  /**
   * Constructor sets main_message to "us"
   * @param to_be_checked_arg Optional argument which is needed for the command
   */
  ComENC();
  ComENC & calibration(uint32_t cm);
  [[nodiscard]] std::string ReturnMessage() const final;
  void parseAnswer(std::string_view message) final;

private:
  uint32_t caliValue_;
};
#endif  // COMMAND_HPP
