#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <string_view>
#include <string>
#include <mutex>
#include <condition_variable>
#include <future>
#include <memory>
#include <utility>
#include "psaf_ucbridge/configuration/static_config.hpp"


/**
 * A Message object contains the real message that is send
 * to the port
 * @brief A Message object contains the real message that is send to the port
 */
class Message
{
protected:
  CommandElement::MessageType type_;
  /**
   * The first character of the message
   */
  const char startchar;

  /**
   * The last character of the messsage
   */
  const char endchar;

  /**
   * The main part of the message
   */
  const std::string & main_message;

  std::future<void> get_ready;

public:
  using SharedPtr = std::shared_ptr<Message>;
  /**
   * Default constructor
   */
  Message(
    char startchar, const std::string & main_message, bool requiresAnswer = true,
    CommandElement::MessageType type = CommandElement::MessageType::WAIT)
  : type_(type),
    startchar(startchar),
    endchar(CommandElement::command_element().kMessageEndChar), main_message(main_message),
    requiresAnswer_(requiresAnswer) {}

  CommandElement::MessageType getType() {return type_;}

  void setType(CommandElement::MessageType type)
  {
    type_ = type;
  }
  void setFuture(std::future<void> && future)
  {
    get_ready = std::move(future);
  }

  /**
  * Indicates if an answer is required from the uc board.
  * @return true if answer is required from uc board
  */
  [[nodiscard]] bool requiresAnswer() const
  {
    return requiresAnswer_;
  }

  /**
   * Adds all member variables together and returns it as one string
   * @return The whole message
   */
  [[nodiscard]] virtual std::string ReturnMessage() const = 0;

  /**
  * Returns true is this command was successful, false otherwise. Defaults to false
  * @return indication if command was successful
  */
  [[nodiscard]] bool success(std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
  {
    std::unique_lock<std::mutex> lk(m_);

    switch (type_) {
      case CommandElement::MessageType::WAIT:
        cv_.wait(lk, [&] {return get_ready.valid();});
        if (get_ready.wait_for(timeout) == std::future_status::ready) {
          return !errorCode_ && (!requiresAnswer() || success_);
        } else {
          return false;
        }

      case CommandElement::MessageType::DISCARD:
        return true;

      case CommandElement::MessageType::ASYNC:
        cv_.wait(lk, [&] {return get_ready.valid();});
        if (get_ready.wait_for(timeout) == std::future_status::ready) {
          return !errorCode_ && (!requiresAnswer() || success_);
        } else {
          return false;
        }

//      default:
//        return false;
    }
  }

  /**
   *
   * @return error code of the last message that was parsed or 0 if there was no error
   */
  [[nodiscard]] int errorCode() const
  {
    return errorCode_;
  }

  /**
   * Get error message which was received after the last command
   * @return string_view of the error message
   */
  std::string & errorMessage()
  {
    return errorMessage_;
  }

  /**
 * Parse the data retrieved by the uc board
 * @param message string retrieved from the uc board
 */
  virtual void parseAnswer(std::string_view message) = 0;

  void notify_one() {cv_.notify_one();}

protected:
  void setRequiresAnswer(bool requires)
  {
    requiresAnswer_ = requires;
  }

  std::mutex m_;
  std::condition_variable cv_;
  /**
* Indicates that last command was successful
*/
  bool success_ = false;

  /**
   * Parses error message and error code
   * @param message string_view of the message from the uc board
   * @return
   */
  int parseErrorMessage(std::string_view message)
  {
    errorMessage_ = "";
    errorCode_ = 0;
    if (message.find(":ERR") == std::string_view::npos) {
      return 0;
    }
    // first get error message
    size_t index = message.find(':', 1);
    if (index != std::string_view::npos && index + 1 < message.size()) {
      errorMessage_ = std::string(message.substr(index + 1));
    }
    // now parse error code
    index = message.find('(');
    size_t index2 = message.find(')', index + 1);
    std::string_view num_str = message.substr(index + 1, index2 - index - 1);
    std::from_chars(num_str.begin(), num_str.begin() + num_str.length(), errorCode_);
    return errorCode_;
  }

private:
  /**
  * does uc board send an answer if
  */
  bool requiresAnswer_;

  /**
 * error message eventually received from the uc board
 */
  std::string errorMessage_;

  /**
   * error code eventually received from the uc board
   */
  int errorCode_ = 0;
};

#endif  // MESSAGE_HPP
