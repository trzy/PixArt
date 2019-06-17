#ifndef INCLUDED_PACKETS_HPP
#define INCLUDED_PACKETS_HPP

#include <cstdint>
#include <cstring>

#define STATIC_ASSERT_PACKET_SIZE(packet) static_assert(sizeof(packet) % 2 == 0 && sizeof(packet) <= (255*2), #packet " size must be a multiple of 2 and not exceed 255 words")

#pragma pack(push, 1)

enum PacketID: uint8_t
{
  Poke = 0,
  Peek,
  PeekResponse,
  ObjectReportRequest,
  ObjectReport
};

struct packet_header
{
  const uint8_t words;
  const PacketID id;

  size_t size() const
  {
    return words * 2;
  }

  packet_header(PacketID packet_id, size_t packet_bytes)
    : id(packet_id),
      words((packet_bytes + 1) /2)
  {
  }
};

struct poke_packet: public packet_header
{
  const uint8_t bank;
  const uint8_t address;
  const uint8_t data;
  const uint8_t __padding__ = 0;

  poke_packet(uint8_t in_bank, uint8_t in_address, uint8_t in_data)
    : packet_header(PacketID::Poke, sizeof(*this)),
      bank(in_bank),
      address(in_address),
      data(in_data)
  {
  }
};

STATIC_ASSERT_PACKET_SIZE(poke_packet);

struct peek_packet: public packet_header
{
  const uint8_t bank;
  const uint8_t address;

  peek_packet(uint8_t in_bank, uint8_t in_address)
    : packet_header(PacketID::Peek, sizeof(*this)),
      bank(in_bank),
      address(in_address)
  {
  }
};

STATIC_ASSERT_PACKET_SIZE(peek_packet);

struct peek_response_packet: public packet_header
{
  const uint8_t bank = 0;
  const uint8_t address = 0;
  const uint8_t data = 0;
  const uint8_t __padding__ = 0;

  peek_response_packet(uint8_t in_bank, uint8_t in_address, uint8_t in_data)
    : packet_header(PacketID::PeekResponse, sizeof(*this)),
      bank(in_bank),
      address(in_address),
      data(in_data)
  {
  }

  peek_response_packet()
    : packet_header(PacketID::PeekResponse, sizeof(*this))
  {
  }
};

STATIC_ASSERT_PACKET_SIZE(peek_response_packet);

struct object_report_request_packet: public packet_header
{
  object_report_request_packet()
    : packet_header(PacketID::ObjectReportRequest, sizeof(*this))
  {
  }
};

STATIC_ASSERT_PACKET_SIZE(object_report_request_packet);

struct object_report_packet: public packet_header
{
  uint8_t data[256];
  const uint8_t format = 0;
  const uint8_t __reserved__ = 0;

  object_report_packet(const uint8_t *in_data, uint8_t in_format)
    : packet_header(PacketID::ObjectReport, sizeof(*this)),
      format(in_format)
  {
    size_t size = 0;
    switch (format)
    {
    default: break;
    case 1: size = 256; break;
    case 2: size = 96; break;
    case 3: size = 144; break;
    case 4: size = 208; break;
    }
    memcpy(data, in_data, size);
  }

  object_report_packet(uint8_t in_format)
    : packet_header(PacketID::ObjectReport, sizeof(*this)),
      format(in_format)
  {
  }

  object_report_packet()
    : packet_header(PacketID::ObjectReport, sizeof(*this))
  {
  }
};

STATIC_ASSERT_PACKET_SIZE(object_report_packet);

#pragma pack(pop)

#endif  // INCLUDED_PACKETS_HPP
