#ifndef PSAF_UCBRIDGE_READING_THREAD_HPP
#define PSAF_UCBRIDGE_READING_THREAD_HPP

#include <memory>
#include <string>
#include "psaf_ucbridge/communication/message_queue.hpp"
#include "psaf_ucbridge/uc_bridge_thread.hpp"
#include "psaf_ucbridge/logging/logger.hpp"

/**
 * @class ReadingThread readingthread.h
 * @brief The ReadingThread class implements threadded polling of the serial input buffer.
 *
*/
class ReadingThread : public UcBridgeThread
{
public:
  using UniquePtr = std::unique_ptr<ReadingThread>;
  /**
   * @brief ReadingThread constructor.
   * @param[in] syntax Syntax object
   * @param[in] dispatcher pointer to a thread_dispatcher object
  */
  ReadingThread(
    MessageQueue<std::string>::SharedPtr dataQueue,
    MessageQueue<std::string>::SharedPtr messageQueue,
    BoardCommunication::SharedPtr com);

protected:
  void doWork() final;

private:
  MessageQueue<std::string>::SharedPtr dataQueue;
  MessageQueue<std::string>::SharedPtr messageQueue;
  BoardCommunication::SharedPtr com;
  Logger::SharedPtr logger;
};

#endif  // PSAF_UCBRIDGE_READING_THREAD_HPP
