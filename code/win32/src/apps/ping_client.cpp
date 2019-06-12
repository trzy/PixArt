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
  while (arduino.isConnected())
  {
    auto start_time = std::chrono::high_resolution_clock::now();
    uint64_t sent_timestamp = start_time.time_since_epoch().count();
    bool success = arduino.writeSerialPort((char *) &sent_timestamp, sizeof(uint64_t));
    if (!success)
    {
      std::cout << "write failed" << std::endl;
      break;
    }

    uint64_t returned_timestamp = 0;
    /*
    size_t bytes_read = arduino.readSerialPort((char *) &returned_timestamp, sizeof(uint64_t));
    if (bytes_read < 8)
    {
      std::cout << "too few bytes read: " << bytes_read << std::endl;
      break;
    }
    */
    read(arduino, (char *) &returned_timestamp, sizeof(uint64_t));
    
    if (returned_timestamp != sent_timestamp)
    {
      std::cout << "round trip error: " << util::hex(returned_timestamp) << std::endl;
      break;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    uint64_t microseconds = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    std::cout << microseconds << std::endl;
  }
  return 0;
}