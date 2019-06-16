#pragma once
#ifndef INCLUDED_PACKET_READER_HPP
#define INCLUDED_PACKET_READER_HPP

#include "pa_driver/packets.hpp"
#include <functional>
#include <vector>

class packet_reader
{
public:
  packet_reader(const std::function<size_t(uint8_t *, size_t)> &&read_fn, const std::function<void(PacketID, const uint8_t *, size_t)> &&on_packet)
    : m_read_fn(read_fn),
      m_on_packet(on_packet)
  {
  }
  
  void tick()
  {
    // Read header
    if (m_idx < sizeof(packet_header))
    {
      resize_buffer(sizeof(packet_header));
      size_t bytes_required = sizeof(packet_header) - m_idx;
      size_t bytes_received = m_read_fn(&m_buffer[m_idx], bytes_required);
      m_idx += bytes_received;
      if (bytes_received < bytes_required)
      {
        // Full header not yet obtained...
        return;
      }
    }
    
    // Read remainder of packet
    const packet_header *header = reinterpret_cast<const packet_header *>(&m_buffer[0]);
    size_t packet_size = header->words * 2;
    resize_buffer(packet_size);
    size_t bytes_remaining = packet_size - m_idx;
    size_t bytes_received = m_read_fn(&m_buffer[m_idx], bytes_remaining);
    m_idx += bytes_received;
    if (bytes_received < bytes_remaining)
    {
      // Packet not yet obtained
      return;
    }
    
    // Fire callback and reset
    m_on_packet(header->id, m_buffer.data(), packet_size);
    m_idx = 0;
  }

private:
  std::function<size_t(uint8_t *, size_t)> m_read_fn;
  std::function<void(PacketID, const uint8_t *, size_t)> m_on_packet;
  std::vector<uint8_t> m_buffer;
  size_t m_idx = 0;
  
  void resize_buffer(size_t size)
  {
    if (m_buffer.size() < size)
    {
      m_buffer.resize(size);
    }
  }
};

#endif  // INCLUDED_PACKET_READER_HPP