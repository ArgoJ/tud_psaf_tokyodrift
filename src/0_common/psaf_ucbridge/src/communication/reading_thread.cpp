/**
 * @file "Communication/readingthread.cpp"
 * @brief Implementaion of the ReadingThread class.
 *
*/
#include <string>
#include <utility>
#include <algorithm>
#include "psaf_ucbridge/configuration/static_config.hpp"
#include "psaf_ucbridge/communication/board_communication.hpp"
#include "psaf_ucbridge/communication/reading_thread.hpp"
#include "psaf_ucbridge/logging/logger_factory.hpp"
#include "psaf_ucbridge/utils.hpp"

ReadingThread::ReadingThread(
  MessageQueue<std::string>::SharedPtr dataQueue,
  MessageQueue<std::string>::SharedPtr messageQueue,
  BoardCommunication::SharedPtr com)
: dataQueue(std::move(dataQueue)),
  messageQueue(std::move(messageQueue)),
  com(std::move(com)),
  logger(LoggerFactory::instance().getLogger("reading_thread")) {}

void ReadingThread::doWork()
{
  static std::string msg;
  if (msg.size() < 150) {
    msg.resize(150);
  }
  if (com->checkIfDataAvailable()) {
    com->read(msg);

    msg.erase(
      std::remove(
        msg.begin(), msg.end(),
        CommandElement::command_element().kResponseStartChar), msg.end());
    msg.erase(
      std::remove(
        msg.begin(), msg.end(),
        CommandElement::command_element().kResponseEndChar), msg.end());
    // remove whitespaces before and after actual message
    Util::trim(msg);

    // put messages in different queues
    if (msg.find(CommandElement::command_element().kChannelGroupMessagePrefix) == 0 ||
      msg.find(CommandElement::command_element().kTextMessagePrefix) == 0)
    {
      dataQueue->push_back(std::move(msg));
    } else if (msg.find(CommandElement::command_element().kAnswerOnCommandPrefix) == 0) {
      messageQueue->push_back(std::move(msg));
    } else {
      logger->info("Other message from uc board: %s", msg.c_str());
    }
  }
}
