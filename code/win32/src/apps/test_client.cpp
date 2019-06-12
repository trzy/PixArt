#include "serial/serial_port.hpp"
#include <iostream>

int main(int argc, char **argv)
{
  std::cout << "Hello, World!" << std::endl;
    
  serial_port arduino("COM3");
  while (arduino.isConnected())
  {
    char buffer[1025];
    size_t bytes_read = arduino.readSerialPort(buffer, sizeof(buffer) - 1);
    if (bytes_read > 0)
    {
        buffer[bytes_read] = 0;
        std::cout << buffer;
    }
  }
  return 0;
}