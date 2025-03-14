#include <string>
#include "psaf_ucbridge/communication/raw_message.hpp"

std::string RawMessage::ReturnMessage() const
{
  std::stringstream ss;
  ss << raw_message_ << endchar;
  return ss.str();
}

void RawMessage::parseAnswer(std::string_view data)
{
  success_ = false;
  if (parseErrorMessage(data)) {
    return;
  }
  raw_answer_ = data;
  success_ = true;
}

std::string RawMessage::getAnswer() const
{
  return raw_answer_;
}
