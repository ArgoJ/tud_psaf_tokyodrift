#ifndef PSAF_UCBRIDGE__SENSOR_GROUP_HPP_
#define PSAF_UCBRIDGE__SENSOR_GROUP_HPP_

#include <string_view>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <stdexcept>
#include <map>
#include "psaf_ucbridge/communication/command.hpp"
#include "psaf_ucbridge/sensor_groups/channel.hpp"
#include "psaf_ucbridge/configuration/static_config.hpp"

/**
 * Group data channels for publishing.
 */
class SensorGroup
{
public:
  using UniquePtr = std::unique_ptr<SensorGroup>;

  /**
   * Instantiates new sensor group. The sensor group is configured according to the message given.
   * @param message ComDAQ object which specifies how the communication to the board is established
   */
  explicit SensorGroup(const ComDAQ & message);

  /**
    * Publish data on a way specified in the implementation.
    * data can be in ASCII format (<value> | <value> [| <value>]),
    * HEX (1234567890ABCDEF) or
    * B64 encoded. The encoding has to be the same as specified in the constructor.
    * Default encoding is ASCII.
    * @param data string with data in encoding specified
    * @return true if data was published, false otherwise
    */
  bool publishData(std::string_view data);

  /**
   *
   * @return
   */
  [[nodiscard]] uint16_t getSensorGroupNo() const
  {
    return groupNo;
  }

  /**
   *
   * @param index
   * @return
   */
  float operator[](size_t index);

  /**
   *
   * @param index
   * @return
   */
  int operator()(size_t index);

  /**
   * What should be done, when data comes
   * @param callback
   */
  void registerCallback(std::function<void(SensorGroup &)> callback);

  void unregisterCallback();

  unsigned int numberOfChannels()
  {
    return channels.size();
  }

protected:
  /**
   * Register all channels defined in the DAQ message in this sensor group
   * @param message ComDAQ which defines the list of channels
   */
  void registerChannels(const ComDAQ & message);

  /**
   *
   */
  CommandElement::DAQEncoding encoding_;

private:
  std::function<void(SensorGroup &)> main_callback_ = nullptr;

  /**
  *
  * @param to_parse
  */
  void parseHEX(std::string_view to_parse);

  /**
 * Parses a string in the format "123 | -456 | 789 [| <num>]"
 * to an array of (signed integers). If There are less numbers than specified in count,
 * the rest of the fields in values is set to 0.
 * @param values
 * @param count
 * @param to_parse
 */
  void parseASCII(std::string_view to_parse);

  /**
   *
   * @param to_parse
   */
  void parseB64(std::string_view to_parse);

  /**
   *
   * @param channel
   */
  void addChannel(GenericChannel::UniquePtr channel);

  /**
   *
   * @tparam T
   * @param channelName
   * @param conversionFactor
   * @return
   */
  template<class T>
  static GenericChannel::UniquePtr createChannel(
    const std::string & channelName,
    float conversionFactor)
  {
    return std::make_unique<Channel<T>>(channelName, conversionFactor);
  }

  /**
   * Convert from channel enum to channel data structure.
   */
  inline static const std::map<CommandElement::ChannelEnum,
    std::function<GenericChannel::UniquePtr()>>
  typeToChannel =
    std::map<CommandElement::ChannelEnum, std::function<GenericChannel::UniquePtr()>>{
    {CommandElement::ChannelEnum::AX, []() {
        return createChannel<int16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::AX),
          CommandElement::kAxConversionFactor);
      }},
    {CommandElement::ChannelEnum::AY, []() {
        return createChannel<int16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::AY),
          CommandElement::kAyConversionFactor);
      }},
    {CommandElement::ChannelEnum::AZ, []() {
        return createChannel<int16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::AZ),
          CommandElement::kAzConversionFactor);
      }},
    {CommandElement::ChannelEnum::GX, []() {
        return createChannel<int16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::GX),
          CommandElement::kWxConversionFactor);
      }},
    {CommandElement::ChannelEnum::GY, []() {
        return createChannel<int16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::GY),
          CommandElement::kWyConversionFactor);
      }},
    {CommandElement::ChannelEnum::GZ, []() {
        return createChannel<int16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::GZ),
          CommandElement::kWzConversionFactor);
      }},
    {CommandElement::ChannelEnum::MX, []() {
        return createChannel<int16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::MX),
          CommandElement::kMxConversionFactor);
      }},
    {CommandElement::ChannelEnum::MY, []() {
        return createChannel<int16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::MY),
          CommandElement::kMyConversionFactor);
      }},
    {CommandElement::ChannelEnum::MZ, []() {
        return createChannel<int16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::MZ),
          CommandElement::kMzConversionFactor);
      }},
    {CommandElement::ChannelEnum::US1, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::US1),
          CommandElement::kUltrasonicConversionFactor);
      }},
    {CommandElement::ChannelEnum::US2, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::US2),
          CommandElement::kUltrasonicConversionFactor);
      }},
    {CommandElement::ChannelEnum::US3, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::US3),
          CommandElement::kUltrasonicConversionFactor);
      }},
    {CommandElement::ChannelEnum::US4, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::US4),
          CommandElement::kUltrasonicConversionFactor);
      }},
    {CommandElement::ChannelEnum::US5, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::US5),
          CommandElement::kUltrasonicConversionFactor);
      }},
    {CommandElement::ChannelEnum::US6, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::US6),
          CommandElement::kUltrasonicConversionFactor);
      }},
    {CommandElement::ChannelEnum::US7, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::US7),
          CommandElement::kUltrasonicConversionFactor);
      }},
    {CommandElement::ChannelEnum::US8, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::US8),
          CommandElement::kUltrasonicConversionFactor);
      }},
    {CommandElement::ChannelEnum::USF, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::USF),
          CommandElement::kUltrasonicConversionFactor);
      }},
    {CommandElement::ChannelEnum::USL, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::USL),
          CommandElement::kUltrasonicConversionFactor);
      }},
    {CommandElement::ChannelEnum::USR, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::USR),
          CommandElement::kUltrasonicConversionFactor);
      }},
    {CommandElement::ChannelEnum::USB, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::USB),
          CommandElement::kUltrasonicConversionFactor);
      }},
    {CommandElement::ChannelEnum::VSBAT, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::VSBAT),
          CommandElement::kVolatgeConversionFactor);
      }},
    {CommandElement::ChannelEnum::VDBAT, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::VDBAT),
          CommandElement::kVolatgeConversionFactor);
      }},
    {CommandElement::ChannelEnum::HALL_DT, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::HALL_DT),
          CommandElement::kDtConversionFactor);
      }},
    {CommandElement::ChannelEnum::HALL_DT8, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::HALL_DT8),
          CommandElement::kDt8ConversionFactor);
      }},
    {CommandElement::ChannelEnum::HALL_CNT, []() {
        return createChannel<uint8_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::HALL_CNT),
          1.0f);
      }},
    {CommandElement::ChannelEnum::PBA, []() {
        return createChannel<uint8_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::PBA),
          1.0f);
      }},
    {CommandElement::ChannelEnum::PBB, []() {
        return createChannel<uint8_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::PBB),
          1.0f);
      }},
    {CommandElement::ChannelEnum::PBC, []() {
        return createChannel<uint8_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::PBC),
          1.0f);
      }},
    {CommandElement::ChannelEnum::ENC_STEPS, []() {
        return createChannel<uint32_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::ENC_STEPS),
          1.0f);
      }},
    {CommandElement::ChannelEnum::ENC_CMS, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::ENC_CMS),
          1.0f);
      }},
    {CommandElement::ChannelEnum::TICS, []() {
        return createChannel<uint32_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::TICS),
          1.0f);
      }},
    {CommandElement::ChannelEnum::UNK0, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::UNK0),
          1.0f);
      }},
    {CommandElement::ChannelEnum::UNK1, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::UNK1),
          1.0f);
      }},
    {CommandElement::ChannelEnum::UNK2, []() {
        return createChannel<uint16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::UNK2),
          1.0f);
      }},
    {CommandElement::ChannelEnum::UNK3, []() {
        return createChannel<int16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::UNK3),
          1.0f);
      }},
    {CommandElement::ChannelEnum::UNK4, []() {
        return createChannel<int16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::UNK4),
          1.0f);
      }},
    {CommandElement::ChannelEnum::UNK5, []() {
        return createChannel<int16_t>(
          CommandElement::command_element().channelNames.at(CommandElement::ChannelEnum::UNK5),
          1.0f);
      }},
  };

  /**
   *
   */
  uint8_t hex2val[256];

  /**
   *
   */
  uint8_t b642val[256];

  /**
   *
   */
  uint16_t groupNo;

  /**
   *
   */
  std::vector<GenericChannel::UniquePtr> channels;
};

#endif  // PSAF_UCBRIDGE__SENSOR_GROUP_HPP_
