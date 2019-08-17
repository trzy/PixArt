#include "pa_driver/packets.hpp"
#include "arduino/packet_reader.hpp"
#include "pixart/settings.hpp"
#include "serial/i_serial_device.hpp"
#include <cstdio>
#include <map>

static double compute_frame_period_seconds(uint32_t frame_period_reg)
{
  double period_100ns = (double) frame_period_reg;
  double period = period_100ns * 100e-9f;
  return period;
}

pixart::settings read_sensor_settings(i_serial_device *port, bool print_settings)
{
  struct
  {
    uint8_t bank;
    uint8_t reg;
  } registers[] =
  {
    { 0x00, 0x02 }, // product ID
    { 0x00, 0x03 }, //
    { 0x00, 0x0f }, // DSP noise threshold
    { 0x00, 0x0b }, // DSP max area threshold
    { 0x00, 0x0c }, //
    { 0x00, 0x10 }, // DSP orientation ratio
    { 0x00, 0x11 }, // DSP orientation factor
    { 0x00, 0x19 }, // DSP maximum object number
    { 0x01, 0x05 }, // sensor gain 1
    { 0x01, 0x06 }, // sensor gain 2
    { 0x01, 0x0e }, // sensor exposure length
    { 0x01, 0x0f }, //
    { 0x0c, 0x60 }, // interpolated resolution x
    { 0x0c, 0x61 }, //
    { 0x0c, 0x62 }, // interpolated resolution y
    { 0x0c, 0x63 }, //
    { 0x0c, 0x07 }, // frame period
    { 0x0c, 0x08 }, //
    { 0x0c, 0x09 }  //
  };

  // Responses are indexed with a key comprising bank and address
  std::map<uint16_t, uint8_t> values;
  packet_reader reader(
    [&](uint8_t *buffer, size_t size) -> size_t
    {
      return port->read(buffer, size);
    },
    [&](PacketID id, const uint8_t *buffer, size_t size) -> bool
    {
      if (id == PacketID::PeekResponse)
      {
        const peek_response_packet *response = reinterpret_cast<const peek_response_packet *>(buffer);
        uint16_t key = (response->bank << 8) | response->address;
        values.emplace(key, response->data);
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
    port->write(peek);
  }

  // Wait for all responses
  reader.wait_for_packets(num_requests);

  // Decode registers
  uint16_t product_id = (values[0x0003] << 8) | values[0x0002];
  uint16_t max_area_threshold = (values[0x000c] << 8) | values[0x000b];
  uint8_t noise_threshold = values[0x000f];
  uint8_t orientation_ratio = values[0x0010];
  uint8_t orientation_factor = values[0x0011];
  uint8_t max_object_number = values[0x0019];
  uint8_t sensor_gain_1 = values[0x0105];
  uint8_t sensor_gain_2 = values[0x0106];
  uint16_t sensor_exposure_length = (values[0x010f] << 8) | values[0x010e];
  uint16_t interpolated_resolution_x = (values[0x0c61] << 8) | values[0x0c60];
  uint16_t interpolated_resolution_y = (values[0x0c63] << 8) | values[0x0c62];
  uint32_t frame_period_reg = (values[0x0c09] << 16) | (values[0x0c08] << 8) | values[0x0c07];
  double frame_period = compute_frame_period_seconds(frame_period_reg);
  double frame_rate = 1.0f / frame_period;

  // Print
  if (print_settings)
  {
    printf("PAJ7025R2 Settings\n");
    printf("------------------\n");
    printf("Product ID                    = 0x%04x %s\n", product_id, product_id == 0x7025 ? "" : "(unknown device)");
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
    printf("\n");
  }
  
  // Return settings object
  return pixart::settings{ resolution_x: interpolated_resolution_x, resolution_y: interpolated_resolution_y };
}

void write_sensor_settings(i_serial_device *port, const pixart::settings &settings)
{
  poke_packet resolution_x_hi(0x0c, 0x61, (settings.resolution_x >> 8) & 0x0f);
  poke_packet resolution_x_lo(0x0c, 0x60, settings.resolution_x & 0xff);
  poke_packet resolution_y_hi(0x0c, 0x63, (settings.resolution_y >> 8) & 0x0f);
  poke_packet resolution_y_lo(0x0c, 0x62, settings.resolution_y & 0xff);
  port->write(resolution_x_hi);
  port->write(resolution_x_lo);
  port->write(resolution_y_hi);
  port->write(resolution_y_lo);
}
