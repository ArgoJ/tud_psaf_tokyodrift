#include "psaf_ucbridge/communication/request.hpp"
#include <charconv>
#include <string>
#include <sstream>
#include <algorithm>
#include <vector>
#include <iostream>
#include "psaf_ucbridge/configuration/static_config.hpp"
#include "psaf_ucbridge/utils.hpp"

void NumberResponse::parseAnswer(std::string_view data)
{
  success_ = false;
  if (parseErrorMessage(data)) {
    return;
  }

  int tmp = 0;
  bool neg = false;
  bool first = true;
  for (auto it = data.begin(); it < data.end(); it++) {
    char c = *it;
    if (c <= '9' && c >= '0') {
      tmp = 10 * tmp + (c - 48);
      first = false;
    } else if (c == '-' && first) {
      neg = true;
      first = false;
    }
  }
  value_ = neg ? -tmp : tmp;
  success_ = true;
}

ReqVer::ReqVer()
: Request(CommandElement::command_element().kMainMessageVer), major_(0), minor_(0), patch_(0) {}

void ReqVer::parseAnswer(std::string_view data)
{
  success_ = false;
  if (parseErrorMessage(data)) {
    return;
  }
  int * parts[3] = {&major_, &minor_, &patch_};
  int stage = 0;
  int tmp = 0;
  for (auto it = data.begin(); it < data.end() && stage < 3; it++) {
    char c = *it;
    if (c <= '9' && c >= '0') {
      tmp = 10 * tmp + (c - 48);
    } else if (c == '.') {
      *(parts[stage]) = tmp;
      tmp = 0;
      stage++;
    }
  }
  *(parts[2]) = tmp;
  success_ = true;
}

int ReqVer::minor() const
{
  return minor_;
}
int ReqVer::major() const
{
  return major_;
}
int ReqVer::patch() const
{
  return patch_;
}

ReqID::ReqID()
: NumberResponse(CommandElement::command_element().kMainMessageId) {}

int ReqID::id() const
{
  return value_;
}

ReqSID::ReqSID()
: NumberResponse(CommandElement::command_element().kMainMessageSid) {}

int ReqSID::id() const
{
  return value_;
}

ReqTICS::ReqTICS()
: NumberResponse(CommandElement::command_element().kMainMessageTics) {}

int ReqTICS::tics() const
{
  return value_;
}

ReqSTEER::ReqSTEER()
: NumberResponse(CommandElement::command_element().kMainMessageSteer) {}

int ReqSTEER::angle() const
{
  return value_;
}

ReqDRV::ReqDRV()
: NumberResponse(CommandElement::command_element().kMainMessageDrv),
  mode_(CommandElement::DRVMode::NIL), aMode_(CommandElement::DRVMode::NIL), activate_(false) {}

ReqDRV & ReqDRV::dms()
{
  mode_ = CommandElement::DRVMode::DMS;
  return *this;
}

ReqDRV & ReqDRV::direct()
{
  mode_ = CommandElement::DRVMode::DIRECT;
  return *this;
}

CommandElement::DRVMode ReqDRV::aMode() const
{
  return aMode_;
}

int ReqDRV::value() const
{
  return value_;
}

bool ReqDRV::activate() const
{
  return activate_;
}

void ReqDRV::parseAnswer(std::string_view data)
{
  NumberResponse::parseAnswer(data);
  switch (mode_) {
    case CommandElement::DRVMode::NIL:
      if ('F' == data[1]) {aMode_ = CommandElement::DRVMode::FORWARDS;} else if ('B' == data[1]) {
        aMode_ = CommandElement::DRVMode::BACKWARDS;
      } else {success_ = false;}
      break;
    case CommandElement::DRVMode::DMS:
      if (data.size() >= 4) {
        aMode_ = CommandElement::DRVMode::DMS;
        if (data.substr(1, 2) == "ON") {activate_ = true;} else if (data.substr(1, 3) == "OFF") {
          activate_ = false;
        } else {success_ = false;}
      } else {success_ = false;}
      break;
    case CommandElement::DRVMode::DIRECT:
      if ('D' == data[1]) {aMode_ = CommandElement::DRVMode::DIRECT;} else {success_ = false;}
      break;
    default:
      success_ = false;
      break;
  }
}

std::string ReqDRV::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message;
  switch (mode_) {
    case CommandElement::DRVMode::NIL:
      break;
    case CommandElement::DRVMode::DMS:
      ss << CommandElement::command_element().kDrvDms;
      break;
    case CommandElement::DRVMode::DIRECT:
      ss << CommandElement::command_element().kDrvD;
      break;
    default:
      break;
  }
  ss << endchar;
  return ss.str();
}

ReqDAQCH::ReqDAQCH()
: Request(CommandElement::command_element().kMainMessageDaq), channels_(false), name_("NONE") {}

void ReqDAQCH::channels()
{
  channels_ = true;
}

void ReqDAQCH::name(std::string_view name)
{
  channels_ = false;
  name_ = name;
}

std::string ReqDAQCH::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message;
  if (channels_) {ss << " CHS" << endchar;} else {ss << " CH" << " " << name_ << endchar;}
  return ss.str();
}
void ReqDAQCH::parseAnswer(std::string_view data)
{
  success_ = false;
  if (parseErrorMessage(data)) {
    return;
  }

  if (name_ == "NONE" && !channels_) {
    return;
  }

  auto endLineChecker = [](char toCheck) {return '\n' == toCheck;};
  auto findColumnDelimiter = [](char toCheck) {return '|' == toCheck;};
  // check number of channels in first line
  auto endl = std::find_if(data.begin() + 1, data.end(), endLineChecker);
  // only CHS needs this check
  if (data.end() == endl && channels_) {
    return;
  }
  int numberOfChannels = 0;
  auto conversionResult = std::from_chars(data.begin() + 1, endl, numberOfChannels);
  // only CHS needs this check
  if (!Util::conversionValid(conversionResult.ec) && channels_) {
    return;
  }

  auto start = channels_ ? endl + 1 : data.begin() + 1;
  endl = std::find_if(start, data.end(), endLineChecker);
  while (start != data.end() + 1) {
    std::array<std::string_view, 5> parts;
    // now find column separators
    auto first = start;
    auto second = std::find_if(first, endl, findColumnDelimiter);
    int idx = 0;
    // taking out the info within each Delimiter
    while (first != endl + 1 && idx < 5) {
      parts[idx] = data.substr(std::distance(data.begin(), first), std::distance(first, second));
      first = second + 1;
      second = std::find_if(first, endl, findColumnDelimiter);
      idx++;
    }

    channelDescriptions_.emplace_back(
      Util::trim(parts[0]),
      Util::trim(parts[1]),
      Util::trim(parts[2]),
      Util::trim(parts[3]),
      Util::trim(parts[4])
    );

    start = endl + 1;
    endl = std::find_if(start, data.end(), endLineChecker);
  }
  success_ = true;
}

const std::vector<ChannelDescription> & ReqDAQCH::getChannelDescriptions() const
{
  return channelDescriptions_;
}
/**
 * ReqDAQGET
 */

ReqDAQGET::ReqDAQGET()
: Request(CommandElement::command_element().kMainMessageDaq),
  age_(false), tics_(false) {}

void ReqDAQGET::age() {age_ = true;}

void ReqDAQGET::tics() {tics_ = true;}

void ReqDAQGET::addSensor(std::string_view sensor)
{
  sensors_.emplace_back(sensor);
}

const std::vector<std::string> & ReqDAQGET::getSensors() const
{
  return sensors_;
}

const std::vector<int> & ReqDAQGET::getAgeValue() const
{
  return ageValue_;
}

const std::vector<int> & ReqDAQGET::getTicsValue() const
{
  return ticsValue_;
}

const std::vector<int> & ReqDAQGET::getMeasureValue() const
{
  return measureValue_;
}
std::string ReqDAQGET::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message << " GET";
  if (age_) {ss << CommandElement::command_element().kDaqAge;}
  if (tics_) {ss << CommandElement::command_element().kDaqTics;}
  for (const auto & sensor : sensors_) {
    ss << " " << sensor;
  }
  ss << endchar;
  return ss.str();
}
void ReqDAQGET::parseAnswer(std::string_view data)
{
  success_ = false;
  if (parseErrorMessage(data)) {
    return;
  }
  if (sensors_.empty()) {return;}

  size_t valueNum = 1;
  if (age_) {valueNum += 1;}
  if (tics_) {valueNum += 1;}
  size_t start = 0;
  size_t end;

  for (size_t i = 0; i < sensors_.size(); i++) {
    end = data.find('|', start + 1);
    std::string_view substr;
    if (end != std::string::npos) {
      substr = data.substr(start + 1, end - start - 1);
      substr = Util::trim(substr);
    } else {
      substr = data.substr(start + 1, end - start - 1);
      substr = Util::trim(substr);
    }
    start = end;

    size_t subStart = 0;
    std::vector<int> values;
    while (subStart != std::string::npos) {
      size_t subEnd = substr.find(' ', subStart + 1);
      values.emplace_back(std::stoi(std::string(substr.substr(subStart, subEnd))));
      subStart = subEnd;
    }
    if (values.size() != valueNum) {
      return;
    }
    measureValue_.emplace_back(values[0]);
    if (age_) {ageValue_.emplace_back((values[1]));}
    if (tics_) {ticsValue_.emplace_back((values[2]));}
  }
  if (age_ && ageValue_.size() != sensors_.size()) {return;}
  if (tics_ && ticsValue_.size() != sensors_.size()) {return;}
  if (measureValue_.size() != sensors_.size()) {return;}
  success_ = true;
}

ReqDAQGRP::ReqDAQGRP()
: Request(CommandElement::command_element().kMainMessageDaq), grps_(false), num_(-1) {}

void ReqDAQGRP::groups() {grps_ = true;}

void ReqDAQGRP::setNum(int i)
{
  grps_ = false;
  num_ = i;
}

std::vector<std::string> & ReqDAQGRP::getInfo() {return info_;}

std::string ReqDAQGRP::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message;
  (grps_) ? ss << CommandElement::command_element().kDaqGrps : ss <<
    CommandElement::command_element().kDaqGrp << num_;
  ss << endchar;
  return ss.str();
}

void ReqDAQGRP::parseAnswer(std::string_view message)
{
  success_ = false;
  if (parseErrorMessage(message)) {
    return;
  }
  if (grps_ && num_ > 0) {return;}
  auto endLineChecker = [](char toCheck) {return '\n' == toCheck;};
  auto start = std::find_if(message.begin(), message.end(), endLineChecker);
  while (start != message.end()) {
    auto end = std::find_if(start + 1, message.end(), endLineChecker);
    info_.emplace_back(
      Util::trim(
        (end == message.end()) ? message.substr(
          std::distance(message.begin(), start + 1),
          std::distance(start, end)) :
        message.substr(std::distance(message.begin(), start + 1), std::distance(start, end - 1))));
    start = end;
  }
  success_ = true;
}

ReqVOUT::ReqVOUT()
: Request(CommandElement::command_element().kMainMessageVout), status_(false) {}

std::string ReqVOUT::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message << endchar;
  return ss.str();
}
bool ReqVOUT::getStatus() const {return status_;}
void ReqVOUT::parseAnswer(std::string_view message)
{
  success_ = false;
  if (parseErrorMessage(message)) {
    return;
  }
  (message.find("ON") != std::string::npos) ? status_ = true : status_ = false;
  success_ = true;
}

ReqIMU::ReqIMU()
: Request(CommandElement::command_element().kMainMessageImu),
  arange_(-1), afilt_(-1), grange_(-1), gfilt_(-1) {}

std::string ReqIMU::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message << CommandElement::command_element().kImuOpt << endchar;
  return ss.str();
}

int ReqIMU::getArange() const
{
  return arange_;
}
int ReqIMU::getAfilt() const
{
  return afilt_;
}
int ReqIMU::getGrange() const
{
  return grange_;
}
int ReqIMU::getGfilt() const
{
  return gfilt_;
}
void ReqIMU::parseAnswer(std::string_view message)
{
  success_ = false;
  if (parseErrorMessage(message)) {
    return;
  }
  auto start1 = message.find("ARANGE") + 7;
  auto end1 = message.find(' ', start1);
  auto start2 = message.find("AFILT") + 6;
  auto end2 = message.find(' ', start2);
  auto start3 = message.find("GRANGE") + 7;
  auto end3 = message.find(' ', start3);
  auto start4 = message.find("GFILT") + 6;
  auto end4 = message.find(' ', start4);
  arange_ = std::stoi(std::string(message.substr(start1, end1 - start1)));
  afilt_ = std::stoi(std::string(message.substr(start2, end2 - start2)));
  grange_ = std::stoi(std::string(message.substr(start3, end3 - start3)));
  gfilt_ = std::stoi(std::string(message.substr(start4, end4 - start4)));
  success_ = true;
}

ReqMAG::ReqMAG()
: Request(CommandElement::command_element().kMainMessageMag), opt_(true), asa_(false), useasa_(
    false)
{}

std::string ReqMAG::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message;
  (opt_) ? ss << CommandElement::command_element().kMagOpt : ss <<
    CommandElement::command_element().kMagAsaq;
  ss << endchar;
  return ss.str();
}
void ReqMAG::parseAnswer(std::string_view message)
{
  success_ = false;
  if (parseErrorMessage(message)) {
    return;
  }
  if (opt_) {
    if (asa_) {return;}
    (message.find('1') != std::string::npos ) ? useasa_ = true : useasa_ = false;
    success_ = true;
  } else {
    if (!asa_) {return;}
    auto start1 = message.find(' ');
    auto start2 = message.find(' ', start1 + 1);
    asavalue_.emplace_back(std::stoi(std::string(message.substr(1, start1 - 1))));
    asavalue_.emplace_back(std::stoi(std::string(message.substr(start1 + 1, start2 - start1 - 1))));
    asavalue_.emplace_back(std::stoi(std::string(message.substr(start2 + 1))));
    success_ = true;
  }
}

ReqUS::ReqUS()
: Request(CommandElement::command_element().kMainMessageUs), opt_(false), status_(false), gain_(-1),
  range_(-1) {}

void ReqUS::opt() {opt_ = true;}

bool ReqUS::getStatus() const {return status_;}

int ReqUS::getGain() const {return gain_;}

int ReqUS::getRange() const {return range_;}

std::string ReqUS::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message;
  if (opt_) {ss << CommandElement::command_element().kUsOpt;}
  ss << endchar;
  return ss.str();
}
void ReqUS::parseAnswer(std::string_view message)
{
  success_ = false;
  if (parseErrorMessage(message)) {
    return;
  }
  if (!opt_) {
    (message.find("ON") != std::string::npos) ? status_ = true : status_ = false;
  } else {
    auto start1 = message.find('=');
    auto start2 = message.find('=', start1 + 1);
    auto end1 = message.find(' ', start1);
    auto end2 = message.find(' ', start2);
    range_ = std::stoi(std::string(message.substr(start1 + 1, end1 - start1 - 1)));
    gain_ = std::stoi(std::string(message.substr(start2 + 1, end2 - start2 - 1)));
  }
  success_ = true;
}


ReqENC::ReqENC()
: NumberResponse(CommandElement::command_element().kMainMessageEnc) {}

int ReqENC::steps() const
{
  return value_;
}

std::string Request::ReturnMessage() const
{
  std::stringstream ss;
  ss << startchar << main_message << endchar;
  return ss.str();
}

ReqRc::ReqRc()
: Request(CommandElement::command_element().kMianMessageRc), mode_(0) {}

void ReqRc::parseAnswer(std::string_view message)
{
  success_ = false;
  if (parseErrorMessage(message)) {
    return;
  }
  if (message.find("RC_ON") != std::string::npos) {
    mode_ = 1;
  } else if (message.find("RC_OFF") != std::string::npos) {mode_ = 0;} else {return;}
  success_ = true;
}
