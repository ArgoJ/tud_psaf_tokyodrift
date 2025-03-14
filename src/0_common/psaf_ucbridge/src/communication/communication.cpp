#include <string>
#include <utility>
#include <chrono>
#include <memory>
#include "psaf_ucbridge/communication/communication.hpp"
#include "psaf_ucbridge/logging/logger_factory.hpp"


Communication::Communication(
  MessageQueue<std::string>::SharedPtr answerQueue,
  IBoardCommunication::SharedPtr com)
: answerQueue_(std::move(answerQueue)),
  commandQueue_(std::make_shared<MessageQueue<std::pair<Message::SharedPtr,
    std::chrono::microseconds>>>()),
  com(std::move(com)),
  logger(LoggerFactory::instance().getLogger("communication")) {}
//  sync_flag(false) {}

void Communication::sendCommand(const Message::SharedPtr & cmd, std::chrono::microseconds timeout)
{
  auto pair = std::make_pair(cmd, timeout);
  commandQueue_->push_back(std::move(pair));
}

void Communication::parseAnswer(const Message::SharedPtr & msg, const std::string & result)
{
  try {
    msg->parseAnswer(std::string_view(result));
  } catch (std::exception & e) {
    logger->error("Result '%s' produced error '%s'", result.c_str(), e.what());
  }
}

void Communication::doWork()
{
//  if (sync_flag) {
//    logger->warn("Result from UcBoard lost, start sync!");
//    std::string last_result;
//    for (auto i = 0; i < 3; i++) {
//      if (answerQueue_->pop(last_result)) {
//        sync_flag = false;
//        return;
//      }
//    }
//    logger->warn(
//      "After 3 attempts got no result from UcBoard! Result could be lost or there is no result!");
//    sync_flag = false;
//  }
  std::pair<Message::SharedPtr, std::chrono::microseconds> pair;
  if (!commandQueue_->pop(pair)) {return;}
  auto msg = pair.first;
  auto timeout = pair.second;
  std::string result;
  com->write(msg->ReturnMessage());
  auto start = std::chrono::high_resolution_clock::now();
  if (answerQueue_->pop(result, timeout, false)) {
    switch (msg->getType()) {
      case CommandElement::MessageType::DISCARD:
        break;
      case CommandElement::MessageType::WAIT:
        {
          auto finish_promise = std::promise<void>();
          msg->setFuture(finish_promise.get_future());
          parseAnswer(msg, result);
          finish_promise.set_value();
          msg->notify_one();
          break;
        }
      case CommandElement::MessageType::ASYNC:
        auto future = std::async(&Communication::parseAnswer, this, msg, std::move(result));
        msg->setFuture(std::move(future));
        msg->notify_one();
        break;
    }
  } else {
//    sync_flag = true;
    switch (msg->getType()) {
      case CommandElement::MessageType::DISCARD:
        break;
      case CommandElement::MessageType::WAIT:
        {
          auto finish_promise = std::promise<void>();
          msg->setFuture(finish_promise.get_future());
          parseAnswer(msg, ":ERR(666): Got no result from Uc Board");
          msg->notify_one();
          break;
        }

      case CommandElement::MessageType::ASYNC:
        auto future = std::async(
          &Communication::parseAnswer, this, msg,
          ":ERR(666): Got no result from Uc Board");
        msg->setFuture(std::move(future));
        msg->notify_one();
        break;
    }
  }
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  logger->debug("Answer return took %d microseconds", duration.count());
}
