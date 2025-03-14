#include <string>
#include <iostream>
#include <memory>
#include "psaf_ucbridge/communication/board_communication.hpp"

void BoardCommunication::openPort(const std::string & port)
{
  if (serial_port.IsOpen()) {
    return;
  }
  try {
    // Open the Serial Port at the desired hardware port.
    serial_port.Open(port);
  } catch (const LibSerial::OpenFailed &) {
    std::cerr << "The serial port did not open correctly." << std::endl;
    // return EXIT_FAILURE ;
  }

  // Set the baud rate of the serial port.
  serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_921600);

  // Set the number of data bits.
  serial_port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);

  // Turn off hardware flow control.
  serial_port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

  // Disable parity.
  serial_port.SetParity(LibSerial::Parity::PARITY_NONE);

  // Set the number of stop bits.
  serial_port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
}

void BoardCommunication::closePort()
{
  if (serial_port.IsOpen()) {
    serial_port.Close();
  }
}

void BoardCommunication::write(const std::string & user_input)
{
  serial_port.Write(user_input);
}

bool BoardCommunication::checkIfDataAvailable()
{
  return serial_port.GetNumberOfBytesAvailable() > 1;
}

void BoardCommunication::read(std::string & data)
{
  serial_port.ReadLine(data, '\u0003');
}
BoardCommunication::BoardCommunication(ISerialPortConfiguration & configuration)
{
  openPort(configuration.serialPort());
}
BoardCommunication::~BoardCommunication()
{
  closePort();
}

BoardCommunication::SharedPtr BoardCommunication::create(ISerialPortConfiguration & configuration)
{
  return std::shared_ptr<BoardCommunication>(new BoardCommunication(configuration));
}
