#ifndef PSAF_UCBRIDGE__CONFIGURATION_BASE_HPP_
#define PSAF_UCBRIDGE__CONFIGURATION_BASE_HPP_
#include <unordered_map>
#include <string>
#include "psaf_ucbridge/configuration/configuration_item.hpp"
#include "psaf_ucbridge/configuration/configuration_item_base.hpp"

/**
 * The base functions of configuration.
 */
class ConfigurationBase
{
protected:
  template<typename T>
  void readValue(const std::string & id, T & reference)
  {
    if (auto * item = dynamic_cast<ConfigurationItem<T> *>(items[id])) {
      reference = item->get();
    }
  }


  template<typename T>
  void insertValue(const std::string & id, T & value)
  {
    // deallocate old value
    auto it = items.find(id);
    auto item = new ConfigurationItem<T>(value);
    if (it != items.end()) {
      delete it->second;
      it->second = item;
    } else {
      items[id] = item;
    }
  }

  template<typename T>
  void insertValue(const std::string & id, T && value)
  {
    // deallocate old value
    auto it = items.find(id);
    auto item = new ConfigurationItem<T>(value);
    if (it != items.end()) {
      delete it->second;
      it->second = item;
    } else {
      items[id] = item;
    }
  }

private:
  std::unordered_map<std::string, ConfigurationItemBase::RawPtr> items;
};

#endif  // PSAF_UCBRIDGE__CONFIGURATION_BASE_HPP_
