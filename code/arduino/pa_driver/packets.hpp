#ifndef INCLUDED_PACKETS_HPP
#define INCLUDED_PACKETS_HPP

#include <cstdint>

#define STATIC_ASSERT_PACKET_SIZE(packet) static_assert(sizeof(packet) % 2 == 0 && sizeof(packet) <= (255*2), #packet " size must be a multiple of 2 and not exceed 255 words")

#pragma pack(push, 1)

enum PacketID: uint8_t
{
  Poke = 0,
  Peek,
  PeekResponse
};

struct packet_header
{
  const uint8_t words;
  const PacketID id;

  size_t bytes() const
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
    : packet_header(PacketID::Poke, sizeof(poke_packet)),
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
    : packet_header(PacketID::Peek, sizeof(peek_packet)),
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
    : packet_header(PacketID::PeekResponse, sizeof(peek_response_packet)),
      bank(in_bank),
      address(in_address),
      data(in_data)
  {
  }

  peek_response_packet()
    : packet_header(PacketID::PeekResponse, sizeof(peek_response_packet))
  {
  }
};

STATIC_ASSERT_PACKET_SIZE(peek_response_packet);

#pragma pack(pop)

#endif  // INCLUDED_PACKETS_HPP
