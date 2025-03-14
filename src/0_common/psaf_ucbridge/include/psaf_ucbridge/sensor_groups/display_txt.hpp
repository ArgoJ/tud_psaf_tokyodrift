#ifndef PSAF_UCBRIDGE__DISPLAY_TXT_HPP_
#define PSAF_UCBRIDGE__DISPLAY_TXT_HPP_

#include <string_view>
#include <memory>


class DisplayTxt
{
public:
  using UniquePtr = std::unique_ptr<DisplayTxt>;
  bool publishData(std::string_view data);
  void registerCallback(std::function<void(DisplayTxt &)> callback);
  void unregisterCallback();
  bool valid() {return nullptr != main_callback_;}
  std::string_view get_data() {return data_;}

private:
  std::function<void(DisplayTxt & callback)> main_callback_;
  std::string_view data_;
};

#endif  // PSAF_UCBRIDGE__DISPLAY_TXT_HPP_
