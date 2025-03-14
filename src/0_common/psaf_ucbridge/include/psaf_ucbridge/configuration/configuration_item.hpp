#ifndef PSAF_UCBRIDGE__CONFIGURATION_ITEM_HPP_
#define PSAF_UCBRIDGE__CONFIGURATION_ITEM_HPP_

#include "psaf_ucbridge/configuration/configuration_item_base.hpp"
/**
 * Base data structure of configuration item.
 * @tparam T
 */
template<typename T>
class ConfigurationItem final : public ConfigurationItemBase
{
public:
  using RawPtr = ConfigurationItem<T>;

  ~ConfigurationItem() final = default;

  explicit ConfigurationItem(T initialValue)
  : item(initialValue) {}

  T & get()
  {
    return item;
  }

private:
  T item;
};

#endif  // PSAF_UCBRIDGE__CONFIGURATION_ITEM_HPP_
