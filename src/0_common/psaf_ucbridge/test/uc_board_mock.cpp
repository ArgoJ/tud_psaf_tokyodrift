#include <csignal>
#include <future>
#include <chrono>
#include <iostream>
#include <sstream>
#include <memory>
#include <string>
#include <thread>
#include "libserial/SerialPort.h"

std::shared_ptr<std::promise<void>> exitPromise;
bool stop = false;

void signalHandler(int signal)
{
  std::cout << "Got signal" << std::endl;
  exitPromise->set_value();
  stop = true;
}

int main(int argc, char * argv[])
{
  exitPromise = std::make_shared<std::promise<void>>();
  std::signal(SIGINT, signalHandler);
  std::mutex protectPort;

  LibSerial::SerialPort port;
  port.Open("/dev/pts/5");
  // Set the baud rate of the serial port.
  port.SetBaudRate(LibSerial::BaudRate::BAUD_921600);

  // Set the number of data bits.
  port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);

  // Turn off hardware flow control.
  port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

  // Disable parity.
  port.SetParity(LibSerial::Parity::PARITY_NONE);

  // Set the number of stop bits.
  port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

  std::shared_future<void> exitFuture = std::shared_future<void>(exitPromise->get_future());
  // std::future_status status = exitFuture.wait_for(std::chrono::seconds(1));
  int16_t i = 0;
  int32_t send = 0;

  std::thread reader([&port, &protectPort]() {
      while (!stop) {
        std::string input;
        port.ReadLine(input, '\n');

        if (input.find("!DRV F") == 0) {
          std::unique_lock<std::mutex> lck(protectPort);
          std::cout << "Received: " << input << "\n";
          std::string response = "\002:F 50 50\003";
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          port.Write(response);
        }
      }
    });

  for (std::future_status status = exitFuture.wait_for(std::chrono::seconds(1));
    std::future_status::ready != status;
    status = exitFuture.wait_for(std::chrono::seconds(1)))
  {
    std::stringstream ss;
    ss << "\002#0: " << i++ << " | " << i++ << " | " << i++ << " | " << i++ << " | " << i++ <<
      " | " <<
      i++ << "\003";
    {
      std::unique_lock<std::mutex> lck(protectPort);
      port.Write(ss.str());
    }

    std::cout << "Writing data: " << ss.str() << std::endl;
    send = 0;
  }

  reader.join();

  port.Close();
  std::cout << "Program terminated" << std::endl;
}
