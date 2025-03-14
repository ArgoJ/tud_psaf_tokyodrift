#include <string_view>
#include <utility>
#include <vector>
#include <memory>
#include "psaf_ucbridge/sensor_groups/sensor_group.hpp"

SensorGroup::SensorGroup(const ComDAQ & message)
: encoding_(message.encoding()), groupNo(message.groupNo())
{
  // initialize arrays with value of character at character position
  // for hex decoding
  for (int i = 0; i < 16; i++) {
    hex2val[static_cast<int>("0123456789abcdef"[i])] = i;
  }

  for (int i = 0; i < 16; i++) {
    hex2val[static_cast<int>("0123456789ABCDEF"[i])] = i;
  }

  // for base64 decoding
  for (int i = 0; i < 64; i++) {
    b642val[static_cast<int>(
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[i])] = i;
  }

  // initialize channels
  registerChannels(message);
}

bool SensorGroup::publishData(std::string_view data)
{
  int index = data.find(':');
  data = data.substr(index + 1);
  if (nullptr == main_callback_) {
    return false;
  }
  switch (encoding_) {
    case CommandElement::DAQEncoding::ASCII: parseASCII(data);
      break;
    case CommandElement::DAQEncoding::HEX: parseHEX(data);
      break;
    case CommandElement::DAQEncoding::BASE64: parseB64(data);
      break;
  }
  main_callback_(*this);
  return true;
}

void SensorGroup::parseHEX(std::string_view to_parse)
{
  size_t length = to_parse.length();
  std::vector<uint8_t> raw(length / 2);
  uint8_t offset = 4;
  uint8_t val = 0;
  size_t index = 0;
  for (auto it = to_parse.begin(); it < to_parse.end(); ++it) {
    val |= hex2val[static_cast<int>(*it)] << offset;
    if (!offset) {
      raw[index++] = val;
      val = 0;
      offset = 4;
    } else {
      offset = 0;
    }
  }
  auto it = raw.begin();
  for (auto chit = channels.begin(); chit < channels.end(); chit++) {
    (*chit)->readValue(it, raw.end());
  }
}
float SensorGroup::operator[](size_t index)
{
  if (index >= channels.size()) {
    throw std::out_of_range("index exceeds the number of channels");
  }
  return channels[index]->value();
}
int SensorGroup::operator()(size_t index)
{
  if (index >= channels.size()) {
    throw std::out_of_range("index exceeds the number of channels");
  }
  return channels[index]->rawValue();
}
void SensorGroup::registerCallback(std::function<void(SensorGroup &)> callback)
{
  main_callback_ = std::move(callback);
}
void SensorGroup::unregisterCallback()
{
  main_callback_ = nullptr;
}
void SensorGroup::registerChannels(const ComDAQ & message)
{
  for (CommandElement::ChannelEnum c : message.getChannels()) {
    addChannel(typeToChannel.at(c)());
  }
}
void SensorGroup::parseASCII(std::string_view to_parse)
{
  uint32_t i = 0;
  auto it = to_parse.begin();
  int32_t val = 0;
  bool first = true;
  bool neg = false;
  // skip all characters which are not numbers
  while (*it > 57 && *it < 48 && *it != 45) {
    it++;
  }

  while (i < channels.size() && it < to_parse.end()) {
    char c = *it;
    if ('-' == c && first) {
      // negative number
      neg = true;
      first = false;
    } else if (c <= 57 && c >= 48) {
      // digit
      val = val * 10 + (c - 48);
      first = false;
    } else if (!first) {
      // save result
      if (neg) {
        val = -val;
      }
      channels[i]->setRawValue(val);
      i++;
      val = 0;
      first = true;
      neg = false;
    }
    it++;
  }

  if (neg) {
    val = -val;
  }
  channels[i]->setRawValue(val);
  i++;

  for (; i < channels.size(); ++i) {
    channels[i]->setRawValue(0);
  }
}
void SensorGroup::parseB64(std::string_view to_parse)
{
  std::vector<uint8_t> raw;
  uint8_t tmp = 0;
  uint32_t offset = 2;
  for (auto it = to_parse.begin(); it < to_parse.end(); it++) {
    uint8_t c = b642val[static_cast<int>(*it)];
    if (2 == offset) {
      tmp |= (c << offset);
      offset = 4;
    } else if (4 == offset) {
      tmp |= (c >> (8 - offset));
      raw.push_back(tmp);
      tmp = 0;
      tmp |= (c << offset);
      offset = 6;
    } else if (6 == offset) {
      tmp |= (c >> (8 - offset));
      raw.push_back(tmp);
      tmp = 0;
      tmp |= (c << offset);
      offset = 0;
    } else {
      // offset = 0
      tmp |= c;
      raw.push_back(tmp);
      tmp = 0;
      offset = 2;
    }
  }

  if (offset != 2) {
    raw.push_back(tmp);
  }

  auto it = raw.begin();
  for (auto chit = channels.begin(); chit < channels.end(); chit++) {
    (*chit)->readValue(it, raw.end());
  }
}

void SensorGroup::addChannel(GenericChannel::UniquePtr channel)
{
  channels.push_back(std::move(channel));
}
