#ifndef PSAF_UCBRIDGE__LOGGER_FACTORY_HPP_
#define PSAF_UCBRIDGE__LOGGER_FACTORY_HPP_

#include <vector>
#include <mutex>
#include <string>
#include "psaf_ucbridge/logging/logger_backend.hpp"
#include "psaf_ucbridge/logging/logger.hpp"

class LoggerFactory
{
private:
  std::mutex mutex_;
  std::vector<Logger::SharedPtr> loggers;
  LoggerBackend * backend = nullptr;

public:
  Logger::SharedPtr getLogger(std::string && name);

  /**
   * Delete all copy/move constructors and copy/move assignments.
   * Class should be used as singleton.
   */
  ~LoggerFactory() = default;
  LoggerFactory(const LoggerFactory &) = delete;
  LoggerFactory & operator=(const LoggerFactory &) = delete;

private:
  LoggerFactory() = default;

  void registerCallbacks(Logger & logger);

public:
  static LoggerFactory & instance()
  {
    static LoggerFactory f;
    return f;
  }

  void registerLoggerBackend(LoggerBackend * provider);
  void unregisterLoggerBackend();
};
#endif  // PSAF_UCBRIDGE__LOGGER_FACTORY_HPP_
