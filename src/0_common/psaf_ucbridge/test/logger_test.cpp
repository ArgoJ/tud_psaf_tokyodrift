#include <string_view>
#include <string>
#include "gtest/gtest.h"
#include "psaf_ucbridge/logging/logger.hpp"
#include "psaf_ucbridge/logging/logger_factory.hpp"
#include "psaf_ucbridge/logging/logger_backend.hpp"
class TestBackend : public LoggerBackend
{
public:
  LoggerCallbackType getFatalLogger(std::string & name) final
  {
    return [this](std::string & message) {
             lastFatal = message;
           };
  }
  LoggerCallbackType getErrorLogger(std::string & name) final
  {
    return [this](std::string & message) {
             lastError = message;
           };
  }
  LoggerCallbackType getWarnLogger(std::string & name) final
  {
    return [this](std::string & message) {
             lastWarn = message;
           };
  }
  LoggerCallbackType getInfoLogger(std::string & name) final
  {
    return [this](std::string & message) {
             lastInfo = message;
           };
  }
  LoggerCallbackType getDebugLogger(std::string & name) final
  {
    return [this](std::string & message) {
             lastDebug = message;
           };
  }

  std::string lastFatal = "";
  std::string lastError = "";
  std::string lastWarn = "";
  std::string lastInfo = "";
  std::string lastDebug = "";
};

TEST(Logger, fatal) {
  TestBackend back;
  Logger::SharedPtr l = LoggerFactory::instance().getLogger("test");
  LoggerFactory::instance().registerLoggerBackend(&back);

  l->fatal("Test %d %s", 123, "bestanden");
  ASSERT_EQ(back.lastFatal, "Test 123 bestanden");

  LoggerFactory::instance().unregisterLoggerBackend();

  l->fatal("Test %d %s", 123, "nicht bestanden");
  ASSERT_EQ(back.lastFatal, "Test 123 bestanden");
}

TEST(Logger, error) {
  TestBackend back;
  Logger::SharedPtr l = LoggerFactory::instance().getLogger("test");
  LoggerFactory::instance().registerLoggerBackend(&back);

  l->error("Test %d %s", 123, "bestanden");
  ASSERT_EQ(back.lastError, "Test 123 bestanden");

  LoggerFactory::instance().unregisterLoggerBackend();

  l->error("Test %d %s", 123, "nicht bestanden");
  ASSERT_EQ(back.lastError, "Test 123 bestanden");
}

TEST(Logger, debug) {
  TestBackend back;
  Logger::SharedPtr l = LoggerFactory::instance().getLogger("test");
  LoggerFactory::instance().registerLoggerBackend(&back);

  l->debug("Test %d %s", 123, "bestanden");
  ASSERT_EQ(back.lastDebug, "Test 123 bestanden");

  LoggerFactory::instance().unregisterLoggerBackend();

  l->debug("Test %d %s", 123, "nicht bestanden");
  ASSERT_EQ(back.lastDebug, "Test 123 bestanden");
}

TEST(Logger, warn) {
  TestBackend back;
  Logger::SharedPtr l = LoggerFactory::instance().getLogger("test");
  LoggerFactory::instance().registerLoggerBackend(&back);

  l->warn("Test %d %s", 123, "bestanden");
  ASSERT_EQ(back.lastWarn, "Test 123 bestanden");

  LoggerFactory::instance().unregisterLoggerBackend();

  l->warn("Test %d %s", 123, "nicht bestanden");
  ASSERT_EQ(back.lastWarn, "Test 123 bestanden");
}

TEST(Logger, info) {
  TestBackend back;
  Logger::SharedPtr l = LoggerFactory::instance().getLogger("test");
  LoggerFactory::instance().registerLoggerBackend(&back);

  l->info("Test %d %s", 123, "bestanden");
  ASSERT_EQ(back.lastInfo, "Test 123 bestanden");

  LoggerFactory::instance().unregisterLoggerBackend();

  l->info("Test %d %s", 123, "nicht bestanden");
  ASSERT_EQ(back.lastInfo, "Test 123 bestanden");
}
