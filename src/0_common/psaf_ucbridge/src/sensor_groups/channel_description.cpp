#include <string>
#include "psaf_ucbridge/sensor_groups/channel_description.hpp"
#include "psaf_ucbridge/utils.hpp"

ChannelDescription::ChannelDescription(
  std::string & name,
  std::string & description,
  std::string & unit,
  std::string & scanTime,
  std::string & dataType)
: name(name), description(description), unit(unit), scanTime(scanTime), rawType(dataType)
{
  Util::toLowerCase(this->name);
  if (dataTypeMapping.find(dataType) != dataTypeMapping.end()) {
    type = dataTypeMapping[dataType];
  } else {
    type = UNKNOWN;
  }
}
std::string ChannelDescription::toString() const
{
  return name + " - " + std::to_string(type);
}

const std::string & ChannelDescription::getName() const
{
  return name;
}
ChannelDescription::DataType ChannelDescription::getType() const
{
  return type;
}
const std::string & ChannelDescription::getRawType() const
{
  return rawType;
}
const std::string & ChannelDescription::getUnit() const
{
  return unit;
}
const std::string & ChannelDescription::getScanTime() const
{
  return scanTime;
}
const std::string & ChannelDescription::getDescription() const
{
  return description;
}
ChannelDescription::ChannelDescription(
  std::string_view name,
  std::string_view description,
  std::string_view unit,
  std::string_view scanTime,
  std::string_view dataType)
: name(name), description(description), unit(unit), scanTime(scanTime), rawType(dataType)
{
  // Util::toLowerCase(this->name);
  std::string dType(dataType);
  if (dataTypeMapping.find(dType) != dataTypeMapping.end()) {
    type = dataTypeMapping[dType];
  } else {
    type = UNKNOWN;
  }
}
