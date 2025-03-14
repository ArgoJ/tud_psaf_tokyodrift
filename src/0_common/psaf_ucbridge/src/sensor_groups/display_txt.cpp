#include <utility>
#include "psaf_ucbridge/sensor_groups/display_txt.hpp"

bool DisplayTxt::publishData(std::string_view data)
{
  this->data_ = data;
  if (main_callback_ == nullptr) {
    return false;
  } else {
    main_callback_(*this);
    return true;
  }
}

void DisplayTxt::registerCallback(std::function<void(DisplayTxt &)> callback)
{
  this->main_callback_ = std::move(callback);
}

void DisplayTxt::unregisterCallback()
{
  this->main_callback_ = nullptr;
}
