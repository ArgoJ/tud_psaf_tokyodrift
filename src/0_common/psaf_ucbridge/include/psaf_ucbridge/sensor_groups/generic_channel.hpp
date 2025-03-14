#ifndef PSAF_UCBRIDGE__GENERICCHANNEL_HPP_
#define PSAF_UCBRIDGE__GENERICCHANNEL_HPP_

#include <string>
#include <utility>
#include <vector>
#include <memory>

/**
 * Channel interface, which defines behaviors of channels
 * @brief define channels behaviors.
 */
class GenericChannel
{
public:
  using UniquePtr = std::unique_ptr<GenericChannel>;
  explicit GenericChannel(std::string name, int size = 0)
  : name_(std::move(name)), size_(size) {}

  /**
   * Get size of data type, which is sent by the channel
   * @return int. size of data type.
   */
  [[nodiscard]] int size() const
  {
    return size_;
  }

  virtual int rawValue() = 0;

  /**
   * Get name of the channel
   * @return string. Name of the channel.
   */
  std::string & name()
  {
    return name_;
  }

  virtual void setRawValue(int value) = 0;

  virtual void readValue(
    std::vector<uint8_t>::iterator & it,
    std::vector<uint8_t>::iterator end) = 0;

  /**
   * return converted value.
   * @return converted data value.
   */
  [[nodiscard]] float value() const
  {
    return value_;
  }

protected:
  float value_ = 0;

private:
  std::string name_;
  // number of bytes in the data type
  int size_ = 0;
};

#endif  // PSAF_UCBRIDGE__GENERICCHANNEL_HPP_
