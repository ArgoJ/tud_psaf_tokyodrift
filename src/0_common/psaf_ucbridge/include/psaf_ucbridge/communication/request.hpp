#ifndef REQUEST_H
#define REQUEST_H

#include <string>
#include <utility>
#include <vector>
#include "psaf_ucbridge/configuration/static_config.hpp"
#include "psaf_ucbridge/communication/message.hpp"
#include "psaf_ucbridge/sensor_groups/channel_description.hpp"

/**
 * Request extends the Message class by setting a startchar.
 * Request is a special form of a message
 * @brief more specified Message
 */
class Request : public Message
{
public:
  /**
   * Constructor sets startchar to the char set in the configuration which is the same for all
   * request-messages.
   */
  explicit Request(const std::string & main_message)
  : Message(CommandElement::command_element().kRequestStartChar, main_message) {}

  [[nodiscard]] std::string ReturnMessage() const override;

protected:
  std::string opt_arg;

  void SetOptArgument(std::string arg)
  {
    opt_arg = std::move(arg);
  }
};

/**
 * This class represents a request that gets ans number as response
 */
class NumberResponse : public Request
{
public:
  explicit NumberResponse(const std::string & main_message)
  : Request(main_message), value_(0) {}

  /**
 * @inherit
 *
 * Parses number from string with format :<number>
 *
 * @param number as string
 */
  void parseAnswer(std::string_view data) override;

protected:
  int value_;
};

/**
 * Ver Request (Version) with main message "ver"
 * @brief Request Version
 */
class ReqVer : public Request
{
public:
  /**
   * Constructor sets main_message to "ver"
   */
  ReqVer();

  /**
   * @inherit
   *
   * Parses the version number which is written as :<major>.<minor>.<patch>
   *
   * @param data version number as string
   */
  void parseAnswer(std::string_view data) final;

  /**
   *
   * @return major part of the version number
   */
  [[nodiscard]] int major() const;
  /**
   *
   * @return minor part of the version number
   */
  [[nodiscard]] int minor() const;
  /**
   *
   * @return patch part of the version number
   */
  [[nodiscard]] int patch() const;

private:
  /**
   * Parts of the version number <major>.<minor>.<patch>
   */
  int major_, minor_, patch_;
};

/**
 * ID Request with main message "id"
 * @brief Request ID
 */
class ReqID : public NumberResponse
{
public:
  /**
   * Constructor sets main_message to "id"
   */
  ReqID();

  /**
   *
   * @return vehicle id
   */
  [[nodiscard]] int id() const;
};

/**
 * SID Request (Session ID) with main message "sid"
 * @brief Request Session ID
 */
class ReqSID : public NumberResponse
{
public:
  /**
   * Constructor sets main_message to "sid"
   */
  ReqSID();

  /**
   *
   * @return session id
   */
  [[nodiscard]] int id() const;
};

/**
 * TICS Request (Milliseconds passed after (re)start) with main message "tics"
 * @brief Request milliseconds after (re)start
 */
class ReqTICS : public NumberResponse
{
public:
  /**
   * Constructor sets main_message to "tics"
   */
  ReqTICS();

  /**
   *
   * @return tics
   */
  [[nodiscard]] int tics() const;
};

/**
 * STEER Request (steering angle) with main message "steer"
 * @brief Request Steering angle
 */
class ReqSTEER : public NumberResponse
{
public:
  /**
   * Constructor sets main_message to "steer"
   */
  ReqSTEER();

  [[nodiscard]] int angle() const;
};

// TODO(johannes): hier weiterarbeiten
/**
 * DRV Request (Driving speed) with main message "drv"
 * @brief Request Driving speed
 */
class ReqDRV : public NumberResponse
{
public:
  /**
   * Constructor sets main_message to "drv"
   */
  ReqDRV();
  ReqDRV & dms();
  ReqDRV & direct();
  [[nodiscard]] CommandElement::DRVMode aMode() const;
  [[nodiscard]] int value() const;
  [[nodiscard]] bool activate() const;


  [[nodiscard]] std::string ReturnMessage() const final;

  /**
   * @inherit
   *
   * Parses the answer depending on the last message (DIRECT/DMS)
   * @param data
   */
  void parseAnswer(std::string_view message) final;

private:
  CommandElement::DRVMode mode_;
  CommandElement::DRVMode aMode_;  // answer mode
  bool activate_;
};

/**
 * DAQ Request (Data Acquisition) with main message "daq"
 * @brief Request information related with the data acquisition(CH or CHS)
 */
class ReqDAQCH : public Request
{
public:
  /**
   * Constructor sets main_message to "daqCH(S)"
   */
  ReqDAQCH();

  void channels();

  void name(std::string_view name);

  [[nodiscard]] std::string ReturnMessage() const final;

  void parseAnswer(std::string_view message) final;

  [[nodiscard]] const std::vector<ChannelDescription> & getChannelDescriptions() const;

private:
  bool channels_;
  std::string name_;
  std::vector<ChannelDescription> channelDescriptions_;
};

/**
 * DAQ Request (Data Acquisition) with main message "daq"
 * @brief Request information related with the data acquisition(GET)
 */
class ReqDAQGET : virtual public Request
{
public:
  /**
   * Constructor sets main_message to "daqGET";
   */
  ReqDAQGET();
  [[nodiscard]] std::string ReturnMessage() const final;
  void age();
  void tics();
  void addSensor(std::string_view sensor);
  [[nodiscard]] const std::vector<std::string> & getSensors() const;
  [[nodiscard]] const std::vector<int> & getAgeValue() const;
  [[nodiscard]] const std::vector<int> & getTicsValue() const;
  [[nodiscard]] const std::vector<int> & getMeasureValue() const;
  void parseAnswer(std::string_view message) final;

private:
  bool age_;
  bool tics_;
  std::vector<std::string> sensors_;
  std::vector<int> ageValue_;
  std::vector<int> ticsValue_;
  std::vector<int> measureValue_;
};

/**
 * DAQ Request (Data Acquisition) with main message "daq"
 * @brief Request information related with the data acquisition(GRP)
 */
class ReqDAQGRP : virtual public Request
{
public:
  ReqDAQGRP();
  [[nodiscard]] std::string ReturnMessage() const final;
  void groups();
  void setNum(int i);
  std::vector<std::string> & getInfo();
  void parseAnswer(std::string_view message) final;

private:
  bool grps_;
  int num_;
  std::vector<std::string> info_;
};

/**
 * VOUT Request (12V Output) with main message "vout"
 * @brief Request state of 12V Output
 */
class ReqVOUT : virtual public Request
{
public:
  /**
   * Constructor sets main_message to "vout"
   */
  ReqVOUT();
  [[nodiscard]] std::string ReturnMessage() const final;
  void parseAnswer(std::string_view message) final;
  [[nodiscard]] bool getStatus() const;

private:
  bool status_;
};

/**
 * IMU Request with main message "imu"
 * @brief Request IMU
 */
class ReqIMU : virtual public Request
{
public:
  ReqIMU();
  [[nodiscard]] std::string ReturnMessage() const final;
  void parseAnswer(std::string_view message) final;
  [[nodiscard]] int getArange() const;
  [[nodiscard]] int getAfilt() const;
  [[nodiscard]] int getGrange() const;
  [[nodiscard]] int getGfilt() const;

private:
  int arange_;
  int afilt_;
  int grange_;
  int gfilt_;
};

/**
 * MAG Request (magnetometer) with main message "mag"
 * @brief Request Magnetometer data
 */
class ReqMAG : virtual public Request
{
public:
  /**
   * Constructor sets main_message to "mag"
   * @param to_be_checked_arg String that should be the optional argument
   */
  ReqMAG();
  void opt()
  {
    opt_ = true;
    asa_ = false;
  }
  void asa()
  {
    asa_ = true;
    opt_ = false;
  }
  [[nodiscard]] std::string ReturnMessage() const final;
  void parseAnswer(std::string_view message) final;
  [[nodiscard]] bool getUseasa() const {return useasa_;}
  const std::vector<int> & getAsavalue() {return asavalue_;}

private:
  bool opt_;
  bool asa_;
  bool useasa_;
  std::vector<int> asavalue_;
};

/**
 * US Request (ultrasonic sensor) with main message "us"
 * @brief Request Ultrasonic sensor data
 */
class ReqUS : virtual public Request
{
public:
  /**
   * Constructor sets main_message to "us"
   */
  ReqUS();
  void opt();
  [[nodiscard]] std::string ReturnMessage() const final;
  void parseAnswer(std::string_view message) final;
  [[nodiscard]] bool getStatus() const;
  [[nodiscard]] int getGain() const;
  [[nodiscard]] int getRange() const;

private:
  bool opt_;
  bool status_;
  int gain_;
  int range_;
};

/**
 * US Request (Incremental encoder) with main message "ENC"
 * @brief Request Incremental encoder data
 */
class ReqENC : virtual public NumberResponse
{
public:
  /**
   * Constructor sets main_message to "enc"
   */
  ReqENC();
  [[nodiscard]] int steps() const;
};

class ReqRc : virtual public Request
{
public:
  ReqRc();
  void parseAnswer(std::string_view message) final;
  int8_t get_mode() {return mode_;}

private:
  int8_t mode_;
};
#endif  // REQUEST_H
