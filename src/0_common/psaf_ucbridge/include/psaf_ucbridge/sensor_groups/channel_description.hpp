#ifndef PSAF_UCBRIDGE__CHANNEL_DESCRIPTION_HPP_
#define PSAF_UCBRIDGE__CHANNEL_DESCRIPTION_HPP_

#include <string_view>
#include <string>
#include <unordered_map>

/**
 * A data structure class. Storing channel description from the board.
 * @note Used for creating sensor group and get sensor settings.
 */
class ChannelDescription
{
public:
  explicit ChannelDescription(
    std::string & name,
    std::string & description,
    std::string & unit,
    std::string & scanTime,
    std::string & dataType
  );

  explicit ChannelDescription(
    std::string_view name,
    std::string_view description,
    std::string_view unit,
    std::string_view scanTime,
    std::string_view dataType
  );

  enum DataType
  {
    Int8,
    UInt8,
    Int16,
    UInt16,
    UNKNOWN
  };

  [[nodiscard]] const std::string & getName() const;

  [[nodiscard]] const std::string & getDescription() const;

  [[nodiscard]] const std::string & getUnit() const;

  [[nodiscard]] const std::string & getScanTime() const;

  [[nodiscard]] const std::string & getRawType() const;

  DataType getType() const;

  std::string toString() const;

private:
  std::string name;
  std::string description;
  std::string unit;
  std::string scanTime;
  std::string rawType;
  DataType type;
  static inline std::unordered_map<std::string, DataType> dataTypeMapping = {
    {"I8", Int8},
    {"U8", UInt8},
    {"I16", Int16},
    {"U16", UInt16}
  };
};

#endif  // PSAF_UCBRIDGE__CHANNEL_DESCRIPTION_HPP_
