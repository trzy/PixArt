#pragma once
#ifndef INCLUDED_PACKET_READER_HPP
#define INCLUDED_PACKET_READER_HPP

#include "pa_driver/packets.hpp"
#include <functional>
#include <vector>

class packet_reader
{
public:
  packet_reader(const std::function<size_t(uint8_t *, size_t)> &&read_fn, const std::function<bool(PacketID, const uint8_t *, size_t)> &&on_packet)
    : m_read_fn(read_fn),
      m_on_packet(on_packet)
  {
  }

  void tick()
  {
    try_get_packet();
  }

  void wait_for_packets(size_t count)
  {
    while (count > 0)
    {
      if (try_get_packet())
      {
        count--;
      }
    }
  }

private:
  std::function<size_t(uint8_t *, size_t)> m_read_fn;
  std::function<bool(PacketID, const uint8_t *, size_t)> m_on_packet;
  std::vector<uint8_t> m_buffer;
  const packet_header *m_header = nullptr;
  size_t m_idx = 0;

  // Returns true when complete packet has been received and callback signals
  // that it was the expected one
  bool try_get_packet()
  {
    // Read header
    if (m_idx < sizeof(packet_header))
    {
      resize_buffer_and_update_header_pointer(sizeof(packet_header));
      size_t bytes_required = sizeof(packet_header) - m_idx;
      size_t bytes_received = m_read_fn(&m_buffer[m_idx], bytes_required);
      m_idx += bytes_received;
      if (bytes_received < bytes_required)
      {
        // Full header not yet obtained...
        return false;
      }
    }

    // Read remainder of packet
    size_t packet_size = m_header->size();
    resize_buffer_and_update_header_pointer(packet_size);
    size_t bytes_remaining = packet_size - m_idx;
    size_t bytes_received = m_read_fn(&m_buffer[m_idx], bytes_remaining);
    m_idx += bytes_received;
    if (bytes_received < bytes_remaining)
    {
      // Packet not yet obtained
      return false;
    }

    // Fire callback and reset
    m_idx = 0;
    return m_on_packet(m_header->id, m_buffer.data(), packet_size);
  }

  void resize_buffer_and_update_header_pointer(size_t size)
  {
    if (m_buffer.size() < size)
    {
      m_buffer.resize(size);

      // Resize may have moved memory. Need updated pointer.
      m_header = reinterpret_cast<const packet_header *>(m_buffer.data());
    }
  }
};

#endif  // INCLUDED_PACKET_READER_HPP