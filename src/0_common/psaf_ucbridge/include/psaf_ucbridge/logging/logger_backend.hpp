#ifndef PSAF_UCBRIDGE__LOGGER_BACKEND_HPP_
#define PSAF_UCBRIDGE__LOGGER_BACKEND_HPP_

#include <string>

class LoggerBackend
{
public:
  using LoggerCallbackType = std::function<void (std::string & message)>;

  virtual LoggerCallbackType getFatalLogger(std::string & name) = 0;
  virtual LoggerCallbackType getErrorLogger(std::string & name) = 0;
  virtual LoggerCallbackType getWarnLogger(std::string & name) = 0;
  virtual LoggerCallbackType getInfoLogger(std::string & name) = 0;
  virtual LoggerCallbackType getDebugLogger(std::string & name) = 0;
};

#endif  // PSAF_UCBRIDGE__LOGGER_BACKEND_HPP_
