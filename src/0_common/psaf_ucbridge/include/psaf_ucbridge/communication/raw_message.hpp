#ifndef PSAF_UCBRIDGE__RAW_MESSAGE_HPP_
#define PSAF_UCBRIDGE__RAW_MESSAGE_HPP_

#include <string>
#include <utility>
#include "psaf_ucbridge/communication/message.hpp"
class RawMessage : public Message
{
public:
  explicit RawMessage(std::string raw_message)
  : Message('i', "non"), raw_message_(std::move(raw_message)) {}

  [[nodiscard]] std::string ReturnMessage() const override;
  void parseAnswer(std::string_view data) override;
  [[nodiscard]] std::string getAnswer() const;

private:
  std::string raw_message_;
  std::string raw_answer_;
};

#endif  // PSAF_UCBRIDGE__RAW_MESSAGE_HPP_
