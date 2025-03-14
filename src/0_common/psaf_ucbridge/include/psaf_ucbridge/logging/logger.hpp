#ifndef PSAF_UCBRIDGE__LOGGER_HPP_
#define PSAF_UCBRIDGE__LOGGER_HPP_

#include <string_view>
#include <functional>
#include <memory>
#include <string>
#include <stdexcept>
#include <map>
#include <sstream>
#include <mutex>
#include <iostream>
#include "psaf_ucbridge/logging/log_level.hpp"


class Logger
{
  friend class LoggerFactory;

public:
  using LoggerCallbackType = std::function<void (std::string & message)>;
  using SharedPtr = std::shared_ptr<Logger>;

private:
  explicit Logger(std::string & name);

  std::string name;

  /**
   * @short Regisers new callback for the logging level type specified in t.
   *
   * Possible types are ERROR, INFO, WARN and DEBUG
   *
   * @param t Type of the loglevel
   * @param callback callback where the message should be passed
   */
  void registerCallback(LogLevel t, LoggerCallbackType callback);

  /**
   * Overwrites all callbacks with the default internal callbacks which provide
   * basic logging functionality.
   */
  void unregisterAllCallbacks();

  /**
   * Overwrites the callback specified with [t] with the default logger for this
   * callback type.
   * @param t Type of the loglevel
   */
  void unregisterCallback(LogLevel t);

public:
  std::string & getName()
  {
    return name;
  }

/**
   * @short Log the message [format] as fatal
   * @param format format string
   * @param ...
   */
  template<typename ... Args>
  void fatal(const char * format, Args... args)
  {
    dispatchAndFormat(FATAL, format, args ...);
  }

  /**
   * @short Log the message [format] as error
   * @param format format string
   * @param ...
   */
  template<typename ... Args>
  void error(const char * format, Args... args)
  {
    dispatchAndFormat(ERROR, format, args ...);
  }

  /**
   * @short Log the message [format] as warning
   * @param format format string
   * @param ...
   */
  template<typename ... Args>
  void warn(const char * format, Args... args)
  {
    dispatchAndFormat(WARN, format, args ...);
  }

  /**
   * @short Log the message [format] as info
   * @param format format string
   * @param ...
   */
  template<typename ... Args>
  void info(const char * format, Args... args)
  {
    dispatchAndFormat(INFO, format, args ...);
  }

  /**
   * @short Log the message [format] as debug
   * @param format format string
   * @param ...
   */
  template<typename ... Args>
  void debug(const char * format, Args... args)
  {
    dispatchAndFormat(DEBUG, format, args ...);
  }

private:
  std::mutex mutex_;

  void initializeCallbacks();

  template<typename ... Args>
  void dispatchAndFormat(LogLevel t, const char * format, Args... args)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (callbacks[t] != nullptr) {
      auto message = string_format(format, args ...);
      callbacks[t](message);
    }
  }

  template<typename ... Args>
  std::string string_format(const std::string & format, Args ... args) const
  {
    int size = snprintf(nullptr, 0, format.c_str(), args ...) + 1;  // Extra space for '\0'
    if (size <= 0) {throw std::runtime_error("Error during formatting.");}
    std::unique_ptr<char[]> buf(new char[size]);
    snprintf(buf.get(), size, format.c_str(), args ...);
    return std::string(buf.get(), buf.get() + size - 1);  // We don't want the '\0' inside
  }

  std::array<LoggerCallbackType, 5> callbacks;
};

#endif  // PSAF_UCBRIDGE__LOGGER_HPP_
