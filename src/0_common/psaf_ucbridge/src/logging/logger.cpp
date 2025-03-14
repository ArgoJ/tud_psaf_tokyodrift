#include <mutex>
#include <utility>
#include <string>
#include "psaf_ucbridge/logging/logger.hpp"
#include "psaf_ucbridge/logging/log_level.hpp"

Logger::Logger(std::string & name)
: name(name)
{
  initializeCallbacks();
}

void Logger::registerCallback(
  LogLevel t,
  Logger::LoggerCallbackType callback)
{
  std::unique_lock<std::mutex> lock(mutex_);
  callbacks[t] = std::move(callback);
}

void Logger::unregisterAllCallbacks()
{
  unregisterCallback(FATAL);
  unregisterCallback(ERROR);
  unregisterCallback(WARN);
  unregisterCallback(INFO);
  unregisterCallback(DEBUG);
}

void Logger::unregisterCallback(LogLevel t)
{
  std::unique_lock<std::mutex> lock(mutex_);
  callbacks[t] = nullptr;
}

void Logger::initializeCallbacks()
{
  unregisterAllCallbacks();
}
