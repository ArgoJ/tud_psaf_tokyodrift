#include <thread>
#include <string>
#include <chrono>
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "psaf_ucbridge/communication/message_queue.hpp"

TEST(test_MessageQueue, test_multithread) {
  MessageQueue<std::string> test_obj;
  std::string results[11];
  std::thread t1([&test_obj]() {
      for (int i = 0; i < 10; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        test_obj.push_back(std::to_string(i));
      }
    });
  std::thread t2([&test_obj, &results]() {
      for (int i = 0; i < 11; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        std::string res;
        test_obj.pop(res, std::chrono::milliseconds(10));
        results[i] = res;
      }
    });
  t1.join();
  t2.join();

  ASSERT_THAT(
    results, testing::ElementsAre(
      "0",
      "1",
      "2",
      "3",
      "4",
      "5",
      "6",
      "7",
      "8",
      "9",
      ""));
}

TEST(test_MessageQueue, test_timeout) {
  MessageQueue<std::string> test_obj;
  std::string results[11];
  std::thread t2([&test_obj, &results]() {
      for (int i = 0; i < 11; i++) {
        std::string res;
        test_obj.pop(res, std::chrono::milliseconds(10));
        results[i] = res;
      }
    });
  t2.join();
  ASSERT_THAT(
    results, testing::ElementsAre(
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      ""));
}

TEST(test_MessageQueue, test_empty) {
  MessageQueue<std::string> test_obj;
  ASSERT_TRUE(test_obj.empty());
  for (int i = 0; i < 10; i++) {
    test_obj.push_back(std::to_string(i));
  }
  // test_obj.notifyAll();
  ASSERT_FALSE(test_obj.empty());
  std::string tmp;
  for (int i = 0; i < 10; i++) {
    test_obj.pop(tmp);
  }
  ASSERT_TRUE(test_obj.empty());
}
