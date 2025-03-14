#include <utility>
#include <memory>
#include <string>
#include "psaf_ucbridge/logging/logger_factory.hpp"

Logger::SharedPtr LoggerFactory::getLogger(std::string && name)
{
  std::unique_lock<std::mutex> lock(mutex_);
  auto * ptr = new Logger(name);
  Logger::SharedPtr sPtr = std::shared_ptr<Logger>(ptr);
  loggers.push_back(std::move(sPtr));
  if (nullptr != backend) {
    registerCallbacks(*loggers.back());
  }
  return loggers.back();
}

void LoggerFactory::registerLoggerBackend(LoggerBackend * provider)
{
  std::unique_lock<std::mutex> lock(mutex_);
  backend = provider;
  if (nullptr != backend) {
    for (const Logger::SharedPtr & logger : loggers) {
      registerCallbacks(*logger);
    }
  } else {
    for (Logger::SharedPtr & logger : loggers) {
      logger->unregisterAllCallbacks();
    }
  }
}

void LoggerFactory::registerCallbacks(Logger & logger)
{
  logger.registerCallback(FATAL, backend->getFatalLogger(logger.getName()));
  logger.registerCallback(ERROR, backend->getErrorLogger(logger.getName()));
  logger.registerCallback(WARN, backend->getWarnLogger(logger.getName()));
  logger.registerCallback(INFO, backend->getInfoLogger(logger.getName()));
  logger.registerCallback(DEBUG, backend->getDebugLogger(logger.getName()));
}

void LoggerFactory::unregisterLoggerBackend()
{
  registerLoggerBackend(nullptr);
}
