#ifndef PSAF_UCBRIDGE__UC_BRIDGE_THREAD_HPP_
#define PSAF_UCBRIDGE__UC_BRIDGE_THREAD_HPP_

#include <thread>
#include <mutex>
#include <future>
#include <chrono>

class UcBridgeThread
{
public:
  explicit UcBridgeThread(std::chrono::milliseconds timeout = std::chrono::milliseconds(0))
  : timeout_(timeout) {}

  void startThread()
  {
    future_ = std::shared_future<void>(exit_promise.get_future());
    worker_ = std::thread(&UcBridgeThread::loop, this);
  }

  void stopThread()
  {
    exit_promise.set_value();
    worker_.join();
  }

protected:
  virtual void doWork() = 0;

private:
  void loop()
  {
    while (isFutureActive()) {
      doWork();
    }
  }

  std::thread worker_;

  bool isFutureActive()
  {
    std::future_status status = future_.wait_for(timeout_);
    return status != std::future_status::ready;
  }

  std::chrono::milliseconds timeout_;

  std::promise<void> exit_promise;

  std::shared_future<void> future_;
};

#endif  // PSAF_UCBRIDGE__UC_BRIDGE_THREAD_HPP_
