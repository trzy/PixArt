#include "pa_driver/packets.hpp"
#include "arduino/packet_reader.hpp"
#include "serial/serial_port.hpp"
#include "util/format.hpp"
#include <iostream>
#include <chrono>

int main(int argc, char **argv)
{
  serial_port arduino_port("COM3");
    
  // Send two peek requests for product ID register
  peek_packet peek1(0, 0x02);
  peek_packet peek2(0, 0x03);
  arduino_port.writeSerialPort((char *) &peek1, sizeof(peek1));
  arduino_port.writeSerialPort((char *) &peek2, sizeof(peek2));
  
  // Wait for responses. Reader places responses into vector.
  std::vector<peek_response_packet> responses;
  packet_reader response_reader(
    [&](uint8_t *buffer, size_t size) -> size_t { return arduino_port.readSerialPort((char *) buffer, size); },
    [&](PacketID id, const uint8_t *buffer, size_t size)
    {
      if (id == PacketID::PeekResponse)
      {
        responses.emplace_back(*reinterpret_cast<const peek_response_packet *>(buffer));
      }
    }
  );
  while (responses.size() < 2)
  {
    response_reader.tick();
  }
  
  // Print response
  uint16_t product_code = (responses[1].data << 8) | responses[0].data;
  std::cout << "Product ID = " << util::hex(product_code) << std::endl;
  return 0;
}