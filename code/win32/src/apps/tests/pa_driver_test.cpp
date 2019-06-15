#include "pa_driver/packets.hpp"
#include "serial/serial_port.hpp"
#include "util/format.hpp"
#include <iostream>
#include <chrono>

void read(serial_port &port, char *buffer, size_t length)
{
  size_t bytes_read = 0;
  while (bytes_read < length)
  {
    size_t bytes_remaining = length - bytes_read;
    bytes_read += port.readSerialPort(&buffer[bytes_read], bytes_remaining);
  }
}

int main(int argc, char **argv)
{
  serial_port arduino("COM3");
  
  // Send two peek requests
  peek_packet peek1(0, 0x02);
  peek_packet peek2(0, 0x03);
  arduino.writeSerialPort((char *) &peek1, sizeof(peek1));
  arduino.writeSerialPort((char *) &peek2, sizeof(peek2));
  
  // Wait for responses
  peek_response_packet response1;
  peek_response_packet response2;
  read(arduino, (char *) &response1, sizeof(response1));
  read(arduino, (char *) &response2, sizeof(response2));
  
  // Print response
  uint16_t product_code = (response2.data << 8) | response1.data;
  std::cout << "Product ID = " << util::hex(product_code) << std::endl;
  return 0;
}