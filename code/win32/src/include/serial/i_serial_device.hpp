#pragma once
#ifndef INCLUDED_I_SERIAL_DEVICE_HPP
#define INCLUDED_I_SERIAL_DEVICE_HPP

#include <cstdint>

class i_serial_device
{
public:
  virtual ~i_serial_device()
  {
  }

  virtual uint32_t read(uint8_t *buffer, uint32_t buf_size) = 0;
  virtual bool write(const uint8_t *buffer, uint32_t buf_size) = 0;
  virtual bool is_connected() const = 0;
  
  template <typename T>
  bool write(const T &object)
  {
    return write(reinterpret_cast<const uint8_t *>(&object), sizeof(T));
  }
};

#endif  // INCLUDED_I_SERIAL_DEVICE_HPP
