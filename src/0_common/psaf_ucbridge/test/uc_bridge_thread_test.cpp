#include <functional>
#include <atomic>
#include <chrono>
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "psaf_ucbridge/uc_bridge_thread.hpp"

class TestThread : public UcBridgeThread
{
public:
  void registerCallback(std::function<void()> fun)
  {
    fun_ = fun;
  }

protected:
  void doWork() final
  {
    fun_();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

private:
  std::function<void()> fun_;
};

TEST(test_UcBridgeThread, test_lifecycle) {
  std::atomic_int test = 0;
  TestThread t1;
  t1.registerCallback([&test]() {test++;});
  t1.startThread();
  while (test < 100) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  t1.stopThread();
}
