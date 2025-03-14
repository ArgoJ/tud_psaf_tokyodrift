#ifndef BOARDCOMMUNICATION_HPP_
#define BOARDCOMMUNICATION_HPP_

#include <string>
#include <memory>
#include "libserial/SerialPort.h"
#include "psaf_ucbridge/communication/data_logger.hpp"
#include "psaf_ucbridge/configuration/configuration_interfaces.hpp"
#include "psaf_ucbridge/communication/board_communication_interface.hpp"

/**
 * BoardCommunication offers all operations needed for working
 * with the serial port
 * @brief Offers Operations for the serial port
 */
class BoardCommunication
  : public IBoardCommunication
{
public:
  using SharedPtr = std::shared_ptr<BoardCommunication>;

  /**
   * Create only one instance of BoardCommunication
   * @param configuration
   * @return std::shared_ptr<BoardCommunication>
   */
  static SharedPtr create(ISerialPortConfiguration & configuration);

  /**
   *
   */
  ~BoardCommunication();

  /**
   * sends the users Input to the port
   * @param user_input  String that will be send to the port
   */
  void write(const std::string & user_input) override;

  /**
   * checks if there is still data sent through the port
   * which has not been processed yet
   * @return  true if there are still bytes available
   */
  bool checkIfDataAvailable() override;

  /**
   * returns received data
   * @return  string that was sent back
   */
  void read(std::string & data) override;

private:
  /**
    *
    */
  LibSerial::SerialPort serial_port;

  /**
   *
   */
  explicit BoardCommunication(ISerialPortConfiguration & configuration);

  /**
   *
   */
  BoardCommunication(const BoardCommunication &);

  /**
   *
   * @return
   */
  BoardCommunication & operator=(const BoardCommunication &);

  /**
   * opens port at SERIAL_PORT_2
   */
  void openPort(const std::string & port);

  /**
   * closes the port
   */
  void closePort();
};

#endif  // BOARDCOMMUNICATION_HPP_
