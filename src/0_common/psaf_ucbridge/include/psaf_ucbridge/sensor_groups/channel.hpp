#ifndef PSAF_UCBRIDGE__CHANNEL_HPP_
#define PSAF_UCBRIDGE__CHANNEL_HPP_

#include <string>
#include <vector>
#include "psaf_ucbridge/sensor_groups/generic_channel.hpp"

/**
 * More specified channel inheriting from GenericChannel
 * @tparam T
 */
template<class T>
class Channel : public GenericChannel
{
public:
  explicit Channel(const std::string & name, float conversionFactor)
  : GenericChannel(name, sizeof(T)), kConversionFactor(conversionFactor) {}

  /**
   * return the unconverted raw value
   * @return int. raw value
   */
  int rawValue() final
  {
    return static_cast<int>(raw_value_);
  }

  /**
   * set raw value
   * @param value raw value
   */
  void setRawValue(int value) final
  {
    raw_value_ = static_cast<T>(value);
    value_ = kConversionFactor * static_cast<float>(raw_value_);
  }

  /**
   * read value from memory. And set raw value and value.
   * @param it iterator. Head of the memory.
   * @param end iterator. End of the memory.
   */
  void readValue(
    std::vector<uint8_t>::iterator & it,
    const std::vector<uint8_t>::iterator end) final
  {
    T value = 0;
    // little endian?
    for (int offset = 0; offset < 8 * size() && it < end; offset += 8) {
      value |= ((*it) << offset);
      it++;
    }
    setRawValue(value);
  }

private:
  T raw_value_ = 0;
  const float kConversionFactor;
};

#endif  // PSAF_UCBRIDGE__CHANNEL_HPP_
