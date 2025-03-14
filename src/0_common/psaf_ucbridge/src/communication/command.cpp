#include <charconv>
#include <string>
#include <sstream>
#include <vector>
#include <exception>
#include <functional>
#include <utility>
#include "psaf_ucbridge/configuration/static_config.hpp"
#include "psaf_ucbridge/communication/command.hpp"
#include "psaf_ucbridge/utils.hpp"

/**
 * ComReset
 */
ComReset::ComReset()
: Command(CommandElement::command_element().kMainMessageReset, false) {}

std::string ComReset::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message << endchar;
  return ss.str();
}

void ComReset::parseAnswer(std::string_view message)
{
  success_ = true;
//  std::stringstream ss;
//  ss << "ComReset has no answer: " << message;
//  throw std::logic_error(ss.str());
}

/**
 * ComLED
 */
ComLED::ComLED()
: Command(CommandElement::command_element().kMainMessageLed),
  all_leds_({{CommandElement::LEDType::A, CommandElement::LEDOptions()},
      {CommandElement::LEDType::B, CommandElement::LEDOptions()},
      {CommandElement::LEDType::BLKL, CommandElement::LEDOptions()},
      {CommandElement::LEDType::BLKR, CommandElement::LEDOptions()},
      {CommandElement::LEDType::BRMS, CommandElement::LEDOptions()}}) {}

void ComLED::setMode(CommandElement::LEDType type, CommandElement::LEDMode mode)
{
  auto & opt = all_leds_[type];
  opt.mode = mode;
}

void ComLED::addSequence(CommandElement::LEDType type, uint32_t on_time, uint32_t off_time)
{
  auto & opt = all_leds_[type];
  opt.on_times.push_back(on_time);
  opt.off_times.push_back(off_time);
}

ComLED & ComLED::on(CommandElement::LEDType type)
{
  setMode(type, CommandElement::LEDMode::ON);
  return *this;
}

ComLED & ComLED::off(CommandElement::LEDType type)
{
  setMode(type, CommandElement::LEDMode::OFF);
  return *this;
}

ComLED & ComLED::toggle(CommandElement::LEDType type)
{
  setMode(type, CommandElement::LEDMode::TOGGLE);
  return *this;
}

ComLED & ComLED::blink(CommandElement::LEDType type, uint32_t on_time, uint32_t off_time)
{
  setMode(type, CommandElement::LEDMode::BLINK);
  addSequence(type, on_time, off_time);
  return *this;
}

ComLED & ComLED::invert(CommandElement::LEDType type, bool inverted)
{
  all_leds_[type].inverted = inverted;
  return *this;
}

std::string ComLED::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message;
  for (const auto & [key, opt] : all_leds_) {
    if (opt.mode == CommandElement::LEDMode::NONE) {
      continue;
    } else if (opt.mode == CommandElement::LEDMode::BLINK) {
      ss << CommandElement::type2string.at(key);
      if (opt.inverted) {ss << CommandElement::command_element().kLedInverted;}
      addBlinkSequence(opt, ss);
    } else {
      ss << CommandElement::type2string.at(key) <<
        CommandElement::mode2string.at(opt.mode);
    }
  }
  ss << endchar;
  return ss.str();
}
void ComLED::addBlinkSequence(const CommandElement::LEDOptions & options, std::stringstream & ss)
{
  for (size_t i = 0; i < options.on_times.size(); i++) {
    ss << " " << options.on_times.at(i) << " " << options.off_times.at(i);
  }
}

void ComLED::parseAnswer(std::string_view message)
{
  success_ = false;
  if (parseErrorMessage(message)) {
    return;
  }
  if (message.find(":ok") != std::string_view::npos) {
    success_ = true;
  }
}

/**
 * ComSID
 */
ComSID::ComSID()
: Command(CommandElement::command_element().kMainMessageSid) {}

ComSID & ComSID::sid(uint32_t sid)
{
  sid_ = sid;
  return *this;
}

std::string ComSID::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message << " " << sid_ << endchar;
  return ss.str();
}

void ComSID::parseAnswer(std::string_view message)
{
  success_ = false;
  if (parseErrorMessage(message)) {
    return;
  }
  int sid;
  auto result = std::from_chars(message.data() + 1, message.data() + message.length(), sid);
  if (Util::conversionValid(result.ec) && static_cast<uint32_t>(sid) == sid_) {
    success_ = true;
  }
}

/**
 * ComSTEER
 */
ComSTEER::ComSTEER()
: Command(CommandElement::command_element().kMainMessageSteer) {}

ComSTEER & ComSTEER::steer(int angle)
{
  angle_ = Util::saturate(angle, CommandElement::kMinSteerValue, CommandElement::kMaxSteerValue);
  return *this;
}

ComSTEER & ComSTEER::repeat(bool repeat)
{
  repeat_ = repeat;
  return *this;
}

std::string ComSTEER::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message << " " << angle_;
  if (repeat_) {
    ss << " " << angle_;
  }
  ss << endchar;
  return ss.str();
}
void ComSTEER::parseAnswer(std::string_view message)
{
  success_ = false;
  if (parseErrorMessage(message)) {
    return;
  }
  int angle;
  auto result = std::from_chars(message.begin() + 1, message.begin() + message.size(), angle);
  if (Util::conversionValid(result.ec) && angle == angle_) {
    success_ = true;
  }
}

/**
 * ComDrv
 */
ComDRV::ComDRV()
: Command(CommandElement::command_element().kMainMessageDrv) {}

ComDRV & ComDRV::forwards(int speed)
{
  mode_ = CommandElement::DRVMode::FORWARDS;
  speed_ =
    Util::saturate(speed, CommandElement::kMinDrivingSpeed, CommandElement::kMaxDrivingSpeed);
  return *this;
}

ComDRV & ComDRV::backwards(int speed)
{
  mode_ = CommandElement::DRVMode::BACKWARDS;
  speed_ = Util::saturate(
    speed, CommandElement::kMinDrivingSpeedBackwards,
    CommandElement::kMaxDrivingSpeedBackwards);
  return *this;
}
ComDRV & ComDRV::direct(int speed)
{
  mode_ = CommandElement::DRVMode::DIRECT;
  speed_ = speed;
  return *this;
}
ComDRV & ComDRV::off()
{
  mode_ = CommandElement::DRVMode::OFF;
  return *this;
}
ComDRV & ComDRV::on()
{
  mode_ = CommandElement::DRVMode::ON;
  return *this;
}
ComDRV & ComDRV::dms(int time)
{
  mode_ = CommandElement::DRVMode::DMS;
  dms_time_ = time;
  return *this;
}
std::string ComDRV::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message;
  switch (mode_) {
    case CommandElement::DRVMode::FORWARDS: ss << CommandElement::command_element().kDrvF << " " <<
        speed_;
      if (repeat_) {
        ss << " " << speed_;
      }
      break;
    case CommandElement::DRVMode::BACKWARDS: ss << CommandElement::command_element().kDrvB << " " <<
        speed_;
      if (repeat_) {
        ss << " " << speed_;
      }
      break;
    case CommandElement::DRVMode::DIRECT: ss << CommandElement::command_element().kDrvD << " " <<
        speed_;
      if (repeat_) {
        ss << " " << speed_;
      }
      break;
    case CommandElement::DRVMode::ON: ss << CommandElement::command_element().kDrvOn;
      break;
    case CommandElement::DRVMode::OFF: ss << CommandElement::command_element().kDrvOff;
      break;
    case CommandElement::DRVMode::DMS: ss << CommandElement::command_element().kDrvDms << "=" <<
        dms_time_;
      break;
    case CommandElement::DRVMode::NIL: break;
  }
  ss << endchar;
  return ss.str();
}
ComDRV & ComDRV::repeat(bool repeat)
{
  repeat_ = repeat;
  return *this;
}
void ComDRV::parseAnswer(std::string_view message)
{
  success_ = false;
  if (parseErrorMessage(message)) {
    return;
  }
  switch (mode_) {
    case CommandElement::DRVMode::FORWARDS:
      if (message.at(1) != 'F') {
        return;
      }
      break;
    case CommandElement::DRVMode::BACKWARDS:
      if (message.at(1) != 'B') {
        return;
      }
      break;
    case CommandElement::DRVMode::DIRECT:
      if (message.at(1) != 'D') {
        return;
      }
      break;
    case CommandElement::DRVMode::ON:
      if (message == ":ON") {
        success_ = true;
      }
      return;
    case CommandElement::DRVMode::OFF:
      if (message == ":OFF") {
        success_ = true;
      }
      return;
    case CommandElement::DRVMode::DMS:
      {
        auto pos = message.find("~DMS=");
        if (pos != std::string_view::npos) {
          int dms_time;
          auto result =
            std::from_chars(message.data() + pos + 5, message.data() + message.length(), dms_time);
          if (Util::conversionValid(result.ec) && dms_time == dms_time_) {
            success_ = true;
          }
        }
        break;
      }
    case CommandElement::DRVMode::NIL:
      if (message == "ok") {
        success_ = true;
      }
      return;
  }

  // check value gotten after DIRECT/FORWARDS/BACKWARDS
  int speed;
  auto result = std::from_chars(message.data() + 3, message.data() + message.length(), speed);
  if (Util::conversionValid(result.ec) && speed == speed_) {
    success_ = true;
  }
}

/**
 * ComDAQ
 */
ComDAQ::ComDAQ()
: Command(CommandElement::command_element().kMainMessageDaq) {}

ComDAQ & ComDAQ::group(int groupNo)
{
  checkGroupNo(groupNo);
  groupNo_ = groupNo;
  maxwait_ = -1;
  ta_ = -1;
  ts_ = -1;
  age_ = false;
  tics_ = false;
  skip_ = -1;
  crc_ = false;
  avg_ = false;
  encoding_ = CommandElement::DAQEncoding::ASCII;
  groupMode_ = CommandElement::DAQGroupMode::CREATE;
  channels.clear();
  mode_ = CommandElement::DAQMode::GROUP;
  scanningMode_ = CommandElement::DAQScanningMode::ALL;
  return *this;
}

void ComDAQ::checkGroupNo(int groupNo)
{
  if (groupNo < CommandElement::kDaqMinGroupNumber ||
    groupNo > CommandElement::kDaqMaxGroupNumber)
  {
    throw std::out_of_range(
            "Group Number has to be between " +
            std::to_string(CommandElement::kDaqMinGroupNumber) +
            " and " + std::to_string(CommandElement::kDaqMaxGroupNumber));
  }
}

ComDAQ & ComDAQ::start()
{
  mode_ = CommandElement::DAQMode::START;
  return *this;
}

ComDAQ & ComDAQ::stop()
{
  mode_ = CommandElement::DAQMode::STOP;
  return *this;
}

ComDAQ & ComDAQ::remove()
{
  groupMode_ = CommandElement::DAQGroupMode::DELETE;
  return *this;
}

ComDAQ & ComDAQ::deactivate()
{
  groupMode_ = CommandElement::DAQGroupMode::DEACTIVATE;
  return *this;
}

ComDAQ & ComDAQ::activate()
{
  groupMode_ = CommandElement::DAQGroupMode::ACTIVATE;
  return *this;
}

ComDAQ & ComDAQ::channel(CommandElement::ChannelEnum c)
{
  channels.push_back(c);
  return *this;
}

ComDAQ & ComDAQ::all(int maxwait)
{
  scanningMode_ = CommandElement::DAQScanningMode::ALL;
  maxwait_ = maxwait;
  return *this;
}

ComDAQ & ComDAQ::any()
{
  scanningMode_ = CommandElement::DAQScanningMode::ANY;
  return *this;
}

ComDAQ & ComDAQ::ts(int ts)
{
  scanningMode_ = CommandElement::DAQScanningMode::TS;
  ts_ = ts;
  return *this;
}

ComDAQ & ComDAQ::avg(int ta)
{
  avg_ = true;
  ta_ = ta;
  return *this;
}

ComDAQ & ComDAQ::skip(int number)
{
  skip_ = number;
  return *this;
}

ComDAQ & ComDAQ::encoding(CommandElement::DAQEncoding encoding)
{
  encoding_ = encoding;
  crc_ = false;
  return *this;
}

ComDAQ & ComDAQ::crc()
{
  crc_ = true;
  return *this;
}

ComDAQ & ComDAQ::age()
{
  age_ = true;
  return *this;
}

ComDAQ & ComDAQ::tics()
{
  tics_ = true;
  return *this;
}

void ComDAQ::addGroupOptions(std::stringstream & ss) const
{
  switch (groupMode_) {
    // case DELETE: ss << " DELETE";
    case CommandElement::DAQGroupMode::DELETE: ss << CommandElement::command_element().kDaqDelete;
      break;
    case CommandElement::DAQGroupMode::DEACTIVATE: ss <<
        CommandElement::command_element().kDaqDeactivate;
      break;
    case CommandElement::DAQGroupMode::ACTIVATE: ss <<
        CommandElement::command_element().kDaqActivate;
      break;
    case CommandElement::DAQGroupMode::CREATE: break;
  }
}
void ComDAQ::addChannels(std::stringstream & ss) const
{
  for (CommandElement::ChannelEnum c : channels) {
    if (c == CommandElement::ChannelEnum::TICS) {continue;}
    ss << " " << CommandElement::command_element().channelNames.at(c);
  }
}

void ComDAQ::addScanningMode(std::stringstream & ss) const
{
  switch (scanningMode_) {
    case CommandElement::DAQScanningMode::ALL: ss << CommandElement::command_element().kDaqAll;
      if (-1 != maxwait_) {
        ss << "=" << maxwait_;
      }
      break;
    case CommandElement::DAQScanningMode::ANY: ss << CommandElement::command_element().kDaqAny;
      break;
    case CommandElement::DAQScanningMode::TS: ss << CommandElement::command_element().kDaqTs <<
        "=" << ts_;
      if (avg_) {
        ss << CommandElement::command_element().kDaqAvg;
        if (-1 != ta_) {
          ss << "=" << ta_;
        }
      }
      break;
  }
}

void ComDAQ::addChannelOptions(std::stringstream & ss) const
{
  if (-1 != skip_) {
    ss << CommandElement::command_element().kDaqSkip << "=" << skip_;
  }
  if (CommandElement::DAQEncoding::ASCII != encoding_) {
    ss << CommandElement::command_element().kDaqEnc << "=";
    switch (encoding_) {
      case CommandElement::DAQEncoding::BASE64: ss << CommandElement::command_element().kDaqB64;
        break;
      case CommandElement::DAQEncoding::HEX: ss << CommandElement::command_element().kDaqHex;
        break;
      case CommandElement::DAQEncoding::ASCII: ss << CommandElement::command_element().kDaqAscii;
        break;
    }
    if (crc_) {
      ss << CommandElement::command_element().kDaqCrc;
    }
  }
  if (age_) {
    ss << CommandElement::command_element().kDaqAge;
  }

  if (tics_) {
    ss << CommandElement::command_element().kDaqTics;
  }
}

std::string ComDAQ::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message;
  switch (mode_) {
    case CommandElement::DAQMode::START: ss << CommandElement::command_element().kDaqStart;
      break;
    case CommandElement::DAQMode::STOP: ss << CommandElement::command_element().kDaqStop;
      break;
    case CommandElement::DAQMode::GROUP: ss << CommandElement::command_element().kDaqGrp <<
        groupNo_;
      addGroupOptions(ss);
      if (CommandElement::DAQGroupMode::CREATE == groupMode_) {
        if (!otherOpt_.empty()) {ss << " " << otherOpt_;} else {
          addChannels(ss);
          addScanningMode(ss);
          addChannelOptions(ss);
        }
      }
      break;
    case CommandElement::DAQMode::NIL: break;
  }
  ss << endchar;
  return ss.str();
}
const std::vector<CommandElement::ChannelEnum> & ComDAQ::getChannels() const
{
  return channels;
}
void ComDAQ::parseAnswer(std::string_view message)
{
  success_ = false;
  if (parseErrorMessage(message)) {
    return;
  }
  switch (mode_) {
    case CommandElement::DAQMode::START:
      if (message == ":started") {
        success_ = true;
      }
      break;
    case CommandElement::DAQMode::STOP:
      if (message == ":stopped") {
        success_ = true;
      }
      break;
    case CommandElement::DAQMode::GROUP:
      if (message == ":ok") {
        success_ = true;
      }
      break;
    default: break;
  }
}
int ComDAQ::groupNo() const
{
  return groupNo_;
}
CommandElement::DAQEncoding ComDAQ::encoding() const
{
  return encoding_;
}
ComDAQ & ComDAQ::otherOpt(std::string opts)
{
  otherOpt_ = std::move(opts);
  return *this;
}

/**
 * ComVout
 */
ComVOUT::ComVOUT()
: Command(CommandElement::command_element().kMainMessageVout) {}

ComVOUT & ComVOUT::on()
{
  on_ = true;
  return *this;
}

ComVOUT & ComVOUT::off()
{
  on_ = false;
  return *this;
}

std::string ComVOUT::ReturnMessage() const
{
  std::string message;
  message += startchar;
  message += main_message;
  if (on_) {
    message += " ON\n";
  } else {
    message += " OFF\n";
  }
  return message;
}
void ComVOUT::parseAnswer(std::string_view message)
{
  success_ = false;
  if (parseErrorMessage(message)) {
    return;
  }
  if (on_) {
    if (message == ":ON") {
      success_ = true;
    }
  } else {
    if (message == ":OFF") {
      success_ = true;
    }
  }
}

/**
 * ComIMU
 */
ComIMU::ComIMU()
: Command(CommandElement::command_element().kMainMessageImu) {}

ComIMU & ComIMU::arange(CommandElement::IMUAcceleration range)
{
  usingdefault = false;
  arange_ = true;
  arangeValue = range;
  return *this;
}

ComIMU & ComIMU::afilt(CommandElement::IMUAFiltMode mode)
{
  usingdefault = false;
  afilter_ = true;
  afilterValue = mode;
  return *this;
}

ComIMU & ComIMU::gfilt(CommandElement::IMUGFiltMode mode)
{
  usingdefault = false;
  gfilter_ = true;
  gfilterValue = mode;
  return *this;
}

ComIMU & ComIMU::grange(CommandElement::IMURotationRate range)
{
  usingdefault = false;
  grange_ = true;
  grangeValue = range;
  return *this;
}

std::string ComIMU::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message << CommandElement::command_element().kImuOpt;
  if (usingdefault) {
    ss << CommandElement::command_element().kImuArange << "=" << arangeValue;
    ss << CommandElement::command_element().kImuAfilt << "=" << afilterValue;
    ss << CommandElement::command_element().kImuGrange << "=" << grangeValue;
    ss << CommandElement::command_element().kImuGfilt << "=" << gfilterValue;
  } else {
    if (arange_) {ss << CommandElement::command_element().kImuArange << "=" << arangeValue;}
    if (afilter_) {ss << CommandElement::command_element().kImuAfilt << "=" << afilterValue;}
    if (grange_) {ss << CommandElement::command_element().kImuGrange << "=" << grangeValue;}
    if (gfilter_) {ss << CommandElement::command_element().kImuGfilt << "=" << gfilterValue;}
  }
  ss << endchar;
  return ss.str();
}
void ComIMU::parseAnswer(std::string_view message)
{
  success_ = false;
  if (parseErrorMessage(message)) {
    return;
  }
  // tmp func to parseAnswer.
  std::function<int(std::string &&)> func = [&message](std::string && patten) {
      size_t n = patten.size();
      std::size_t p = message.find(patten);
      std::size_t p1 = message.find(' ', p);
      if (p1 == message.npos) {p1 = message.size();}
      int value = std::atoi(message.substr(p + n, p1).data());
      return value;
    };

  if (usingdefault) {
    if (func("~ARANGE=") != arangeValue) {
      return;
    }
    if (func("~AFILT=") != afilterValue) {
      return;
    }
    if (func("~GRANGE=") != grangeValue) {
      return;
    }
    if (func("~GFILT=") != gfilterValue) {
      return;
    }
  } else {
    if (arange_) {
      if (func("~ARANGE=") != arangeValue) {
        return;
      }
    }
    if (afilter_) {
      if (func("~AFILT=") != afilterValue) {
        return;
      }
    }
    if (grange_) {
      if (func("~GRANGE=") != grangeValue) {
        return;
      }
    }
    if (gfilter_) {
      if (func("~GFILT=") != gfilterValue) {
        return;
      }
    }
  }
  success_ = true;
}

/**
 * ComMag
 */
ComMAG::ComMAG()
: Command(CommandElement::command_element().kMainMessageMag) {}

ComMAG & ComMAG::useasa(bool activate)
{
  useasa_ = activate;
  return *this;
}

std::string ComMAG::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message << CommandElement::command_element().kMagOpt;
  if (useasa_) {
    ss << CommandElement::command_element().kMagAsa << "=1";
  } else {
    ss << CommandElement::command_element().kMagAsa << "=0";
  }
  ss << endchar;
  return ss.str();
}
void ComMAG::parseAnswer(std::string_view message)
{
  success_ = false;
  if (parseErrorMessage(message)) {
    return;
  }

  if (useasa_) {
    if (message == ":~USEASA=1") {
      success_ = true;
    }
  } else {
    if (message == ":~USEASA=0") {
      success_ = true;
    }
  }
}

/**
 *ComUS
 */
ComUS::ComUS()
: Command(CommandElement::command_element().kMainMessageUs) {}

ComUS & ComUS::on()
{
  usingdefault = false;
  mode_ = CommandElement::USMode::ON;
  return *this;
}

ComUS & ComUS::off()
{
  usingdefault = false;
  mode_ = CommandElement::USMode::OFF;
  return *this;
}

ComUS & ComUS::gain(uint32_t gain)
{
  usingdefault = false;
  mode_ = CommandElement::USMode::OPT;
  gain_ = true;
  gainValue = gain;
  return *this;
}

ComUS & ComUS::range(uint32_t range)
{
  usingdefault = false;
  mode_ = CommandElement::USMode::OPT;
  range_ = true;
  rangeValue = range;
  return *this;
}

std::string ComUS::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message;
  switch (mode_) {
    case CommandElement::USMode::ON: ss << CommandElement::command_element().kUsOn;
      break;
    case CommandElement::USMode::OFF: ss << CommandElement::command_element().kUsOff;
      break;
    case CommandElement::USMode::OPT: ss << CommandElement::command_element().kUsOpt;
      if (usingdefault) {
        ss << CommandElement::command_element().kUsGain << "=" << gainValue;
        ss << CommandElement::command_element().kUsRange << "=" << rangeValue;
      } else {
        if (gain_) {
          ss << CommandElement::command_element().kUsGain << "=" << gainValue;
        }
        if (range_) {
          ss << CommandElement::command_element().kUsRange << "=" << rangeValue;
        }
      }
  }
  ss << endchar;
  return ss.str();
}

void ComUS::parseAnswer(std::string_view message)
{
  success_ = false;
  if (parseErrorMessage(message)) {
    return;
  }
  switch (mode_) {
    case CommandElement::USMode::ON:
      if (message == ":ON") {
        success_ = true;
      }
      break;
    case CommandElement::USMode::OFF:
      if (message == ":OFF") {
        success_ = true;
      }
      break;
    case CommandElement::USMode::OPT:
      // TODO(johannes): check which message is returned if options are set
      // tmp func to parseAnswer. return the corresponding value.
      std::function<uint32_t(std::string &&)> func = [&message](std::string && patten) {
          size_t n = patten.size();
          std::size_t p = message.find(patten);
          std::size_t p1 = message.find(' ', p);
          if (p1 == message.npos) {p1 = message.size();}
          uint32_t value = std::atoi(message.substr(p + n, p1).data());
          return value;
        };
      if (usingdefault) {
        if (func("~RANGE=") != rangeValue) {
          return;
        }
        if (func("~GAIN=") != gainValue) {
          return;
        }
      } else {
        if (range_) {
          if (func("~RANGE=") != rangeValue) {
            return;
          }
        }
        if (gain_) {
          if (func("~GAIN=") != gainValue) {
            return;
          }
        }
      }
      success_ = true;
      break;
  }
}

ComRaw::ComRaw()
: Command("") {}

std::string ComRaw::ReturnMessage() const
{
  std::stringstream ss;
  ss << raw_message_ << "\n";
  return ss.str();
}
ComRaw & ComRaw::raw(const std::string & raw_message)
{
  raw_message_ = raw_message;
  return *this;
}

void ComRaw::parseAnswer(std::string_view message)
{
  success_ = false;
  if (parseErrorMessage(message)) {
    return;
  }
  if (answer_.empty()) {
    if (message.find(":ok") == 0) {
      success_ = true;
    }
  } else {
    if (message.find(answer_) == 0) {
      success_ = true;
    }
  }
}

ComRaw & ComRaw::answer(const std::string & answer)
{
  answer_ = answer;
  return *this;
}

ComRaw & ComRaw::requireAnswer(bool requires)
{
  setRequiresAnswer(requires);
  return *this;
}

ComENC::ComENC()
: Command(CommandElement::command_element().kMainMessageEnc) {}

ComENC & ComENC::calibration(uint32_t cm)
{
  caliValue_ = cm;
  return *this;
}

std::string ComENC::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message;
  ss << " " << caliValue_ << endchar;
  return ss.str();
}

void ComENC::parseAnswer(std::string_view message)
{
  success_ = false;
  if (parseErrorMessage(message)) {
    return;
  }
  if (message.find(":ok") != std::string_view::npos) {
    success_ = true;
  }
}
