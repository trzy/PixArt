#pragma once
#ifndef INCLUDED_SERIAL_REPLAY_DEVICE_HPP
#define INCLUDED_SERIAL_REPLAY_DEVICE_HPP

#include "serial/i_serial_device.hpp"
#include <memory>
#include <fstream>

/*
 * Records all traffic from a serial device to a file and supports replay.
 * Writes to the serial device are ignored.
 */

class serial_replay_device: public i_serial_device
{
private:
  std::unique_ptr<i_serial_device> m_serial_device;
  const std::string m_filename;
  std::ofstream m_of;
  std::unique_ptr<uint8_t[]> m_buffer;
  size_t m_buffer_size = 0;
  size_t m_read_idx = 0;
  size_t m_next_block_idx = 0;

  bool is_recording() const;
  void sync_to_block_boundary();
  uint32_t read_from_block(uint8_t *buffer, uint32_t buf_size);

public:
  // Record
  serial_replay_device(const std::string &capture_file, std::unique_ptr<i_serial_device> serial_device);

  // Replay
  serial_replay_device(const std::string &replay_file);

  ~serial_replay_device();

  uint32_t read(uint8_t *buffer, uint32_t buf_size) override;
  bool write(const uint8_t *buffer, uint32_t buf_size) override;
  bool is_connected() const override;
};

#endif  // INCLUDED_SERIAL_REPLAY_DEVICE_HPP
