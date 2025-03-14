#ifndef PSAF_UCBRIDGE__CONFIGURATION_ITEM_BASE_HPP_
#define PSAF_UCBRIDGE__CONFIGURATION_ITEM_BASE_HPP_

class ConfigurationItemBase
{
public:
  using RawPtr = ConfigurationItemBase *;
  virtual ~ConfigurationItemBase() = default;
};

#endif  // PSAF_UCBRIDGE__CONFIGURATION_ITEM_BASE_HPP_
