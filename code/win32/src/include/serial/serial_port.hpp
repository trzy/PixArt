#pragma once
#ifndef INCLUDED_SERIAL_PORT_HPP
#define INCLUDED_SERIAL_PORT_HPP

#define ARDUINO_WAIT_TIME 2000
#define MAX_DATA_LENGTH 255

#include "serial/i_serial_device.hpp"
#include <windows.h>
#include <string>

class serial_port: public i_serial_device
{
private:
  static const constexpr DWORD ArduinoWaitTime = 2000;
  static const constexpr size_t MaxDataLength = 255;

  HANDLE m_handler;
  bool m_connected;
  COMSTAT m_status;
  DWORD m_errors;

public:
  serial_port(const std::string &port_name, unsigned baud_rate = 9600);
  ~serial_port();
  uint32_t read(uint8_t *buffer, uint32_t buf_size) override;
  bool write(const uint8_t *buffer, uint32_t buf_size) override;
  bool is_connected() const override;
};

#endif  // INCLUDED_SERIAL_PORT_HPP
