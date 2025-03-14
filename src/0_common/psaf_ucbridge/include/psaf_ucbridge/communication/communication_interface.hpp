#ifndef PSAF_UCBRIDGE__COMMUNICATION_INTERFACE_HPP_
#define PSAF_UCBRIDGE__COMMUNICATION_INTERFACE_HPP_
#include <string>
#include <iostream>
#include <chrono>
#include <memory>
#include "psaf_ucbridge/logging/logger.hpp"
#include "psaf_ucbridge/communication/message.hpp"
#include "psaf_ucbridge/communication/board_communication.hpp"
#include "psaf_ucbridge/communication/message_queue.hpp"

/**
 * Communication interface
 */
class ICommunication
{
public:
  using SharedPtr = std::shared_ptr<ICommunication>;
  virtual void sendCommand(
    const Message::SharedPtr & cmd,
    std::chrono::microseconds timeout) = 0;
};
#endif  //  PSAF_UCBRIDGE__COMMUNICATION_INTERFACE_HPP_
