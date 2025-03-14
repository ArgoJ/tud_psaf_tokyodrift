#ifndef PSAF_UCBRIDGE_COMMUNICATION_HPP
#define PSAF_UCBRIDGE_COMMUNICATION_HPP

#include <string>
#include <iostream>
#include <chrono>
#include <memory>
#include <utility>
#include "psaf_ucbridge/logging/logger.hpp"
#include "psaf_ucbridge/communication/message.hpp"
#include "psaf_ucbridge/communication/board_communication.hpp"
#include "psaf_ucbridge/communication/message_queue.hpp"
#include "psaf_ucbridge/communication/communication_interface.hpp"
#include "psaf_ucbridge/uc_bridge_thread.hpp"
#include "psaf_ucbridge/configuration/static_config.hpp"

class Communication : public ICommunication, public UcBridgeThread
{
public:
  explicit Communication(
    MessageQueue<std::string>::SharedPtr answerQueue,
    IBoardCommunication::SharedPtr com);

  /**
   * Send command to uc_bridge and print response to std-out
   */
  void sendCommand(
    const Message::SharedPtr & cmd,
    std::chrono::microseconds timeout) final;

protected:
  void doWork() final;

private:
  void parseAnswer(const Message::SharedPtr & msg, const std::string & result);
  MessageQueue<std::string>::SharedPtr answerQueue_;
  MessageQueue<std::pair<Message::SharedPtr, std::chrono::microseconds>>::SharedPtr commandQueue_;
  IBoardCommunication::SharedPtr com;
  Logger::SharedPtr logger;
//  bool sync_flag;
};

#endif  // PSAF_UCBRIDGE_COMMUNICATION_HPP
