#ifndef PSAF_UCBRIDGE__BOARD_COMMUNICATION_INTERFACE_HPP_
#define PSAF_UCBRIDGE__BOARD_COMMUNICATION_INTERFACE_HPP_

#include <string>
#include <memory>

/**
 * BoardCommunication interface
 */
class IBoardCommunication
{
public:
  using SharedPtr = std::shared_ptr<IBoardCommunication>;
  virtual void write(const std::string & user_input) = 0;
  virtual bool checkIfDataAvailable() = 0;
  virtual void read(std::string & data) = 0;
};

#endif  // PSAF_UCBRIDGE__BOARD_COMMUNICATION_INTERFACE_HPP_
