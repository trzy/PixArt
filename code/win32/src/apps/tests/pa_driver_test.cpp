#include "pa_driver/packets.hpp"
#include "arduino/packet_reader.hpp"
#include "serial/serial_port.hpp"
#include "util/format.hpp"
#include <cstdio>
#include <chrono>

static double compute_frame_period_seconds(uint32_t frame_period_reg)
{
  double period_100ns = (double) frame_period_reg;
  double period = period_100ns * 100e-9f;
  return period;
}

static void print_sensor_settings(serial_port *port)
{
  struct
  {
    uint8_t bank;
    uint8_t reg;
  } registers[] =
  {
    { 0x00, 0x02 }, // 0  product ID
    { 0x00, 0x03 }, // 1
    { 0x00, 0x0f }, // 2  DSP noise threshold
    { 0x00, 0x10 }, // 3  DSP orientation ratio
    { 0x00, 0x11 }, // 4  DSP orientation factor
    { 0x00, 0x19 }, // 5  DSP maximum object number
    { 0x01, 0x05 }, // 6  sensor gain 1
    { 0x01, 0x06 }, // 7  sensor gain 2
    { 0x01, 0x0e }, // 8  sensor exposure length
    { 0x01, 0x0f }, // 9
    { 0x0c, 0x60 }, // 10 interpolated resolution x
    { 0x0c, 0x61 }, // 11
    { 0x0c, 0x62 }, // 12 interpolated resolution y
    { 0x0c, 0x63 }, // 13
    { 0x00, 0x0b }, // 14 DSP max area threshold
    { 0x00, 0x0c }, // 15
    { 0x0c, 0x07 }, // 16 frame period
    { 0x0c, 0x08 }, // 17
    { 0x0c, 0x09 }  // 18
  };

  std::vector<peek_response_packet> responses;
  packet_reader reader(
    [&](uint8_t *buffer, size_t size) -> size_t
    {
      return port->readSerialPort((char *) buffer, size);
    },
    [&](PacketID id, const uint8_t *buffer, size_t size) -> bool
    {
      if (id == PacketID::PeekResponse)
      {
        responses.emplace_back(*reinterpret_cast<const peek_response_packet *>(buffer));
        return true;
      }
      return false;
    }
  );

  // Send peek request for each register
  size_t num_requests = sizeof(registers) / sizeof(registers[0]);
  for (size_t i = 0; i <num_requests; i++)
  {
    peek_packet peek(registers[i].bank, registers[i].reg);
    port->writeSerialPort((char *) &peek, sizeof(peek));
  }

  // Wait for all responses
  reader.wait_for_packets(num_requests);

  // Decode registers
  uint16_t product_id = (responses[1].data << 8) | responses[0].data;
  uint16_t max_area_threshold = (responses[15].data << 8) | responses[14].data;
  uint8_t noise_threshold = responses[2].data;
  uint8_t orientation_ratio = responses[3].data;
  uint8_t orientation_factor = responses[4].data;
  uint8_t max_object_number = responses[5].data;
  uint8_t sensor_gain_1 = responses[6].data;
  uint8_t sensor_gain_2 = responses[7].data;
  uint16_t sensor_exposure_length = (responses[9].data << 8) | responses[8].data;
  uint16_t interpolated_resolution_x = (responses[11].data << 8) | responses[10].data;
  uint16_t interpolated_resolution_y = (responses[13].data << 8) | responses[12].data;
  uint32_t frame_period_reg = (responses[18].data << 16) | (responses[17].data << 8) | responses[16].data;
  double frame_period = compute_frame_period_seconds(frame_period_reg);
  double frame_rate = 1.0f / frame_period;

  // Print
  printf("PAJ7025R2 Settings\n");
  printf("------------------\n");
  printf("Product ID                    = 0x%04x\n", product_id);
  printf("DSP area max threshold        = 0x%04x\n", max_area_threshold);
  printf("DSP noise threshold           = 0x%02x\n", noise_threshold);
  printf("DSP orientation ratio         = 0x%02x\n", orientation_ratio);
  printf("DSP orientation factor        = 0x%02x\n", orientation_factor);
  printf("DSP maximum number of objects = %d\n", max_object_number);
  printf("Sensor gain 1                 = 0x%02x\n", sensor_gain_1);
  printf("Sensor gain 2                 = 0x%02x\n", sensor_gain_2);
  printf("Sensor exposure length        = 0x%04x\n", sensor_exposure_length);
  printf("Scale resolution X            = %d\n", interpolated_resolution_x);
  printf("Scale resolution Y            = %d\n", interpolated_resolution_y);
  printf("Frame period                  = %1.4f ms\n", frame_period * 1000);
  printf("Frame rate                    = %1.2f Hz\n", frame_rate);
}

int main(int argc, char **argv)
{
  serial_port arduino_port("COM3");
  print_sensor_settings(&arduino_port);
  return 0;
}