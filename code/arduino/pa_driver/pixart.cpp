#include "pixart.hpp"
#include "pixart_object.hpp"
#include <Arduino.h>
#include <SPI.h>

static constexpr uint8_t PIN_CSB = A0;
static uint32_t s_frame_period_micros = 0;

static void chip_select(bool enable)
{
  digitalWrite(PIN_CSB, enable ? 0 : 1);
}

static void write(uint8_t reg, uint8_t data)
{
  SPI.transfer(0x00);     // bit 7 = write (0), bits 6-0 = single byte (0)
  SPI.transfer(reg);
  SPI.transfer(data);
}

static uint8_t read(uint8_t reg)
{
  SPI.transfer(0x80);     // bit 7 = read (1), bits 6-0 = single byte (0)
  SPI.transfer(reg);
  return SPI.transfer(0);
}

static void burst_read(uint8_t reg_base, uint8_t buffer[], uint16_t num_bytes)
{
  SPI.transfer(0x81);
  SPI.transfer(reg_base);
  for (uint16_t i = 0; i < num_bytes; i++)
  {
    buffer[i] = SPI.transfer(0);
  }
}

static void load_initial_settings()
{
  write(0xef, 0);
  write(0xdc, 0);
  write(0xfb, 4);
  write(0xef, 0);
  write(0x2f, 5);
  write(0x30, 0);
  write(0x30, 1);
  write(0x1f, 0);
  write(0xef, 1);
  write(0x2d, 0);
  write(0xef, 0xc);
  write(0x64, 0);
  write(0x65, 0);
  write(0x66, 0);
  write(0x67, 0);
  write(0x68, 0);
  write(0x69, 0);
  write(0x6a, 0);
  write(0x6b, 0);
  write(0x6c, 0);
  write(0x71, 0);
  write(0x72, 0);
  write(0x12, 0);
  write(0x13, 0);
  write(0xef, 0);
  write(0x01, 1);
}

uint32_t PA_get_frame_period_microseconds()
{
  // Read frame period, which is in units of 100 ns
  chip_select(true);
  write(0xef, 0x0c); // bank C
  uint32_t cmd_frame_period = read(0x07);
  cmd_frame_period |= read(0x08) << 8;
  cmd_frame_period |= read(0x09) << 16;
  chip_select(false);

  // 100 ns -> us, rounding to nearest microsecond
  uint32_t microseconds = (cmd_frame_period / 10) + ((cmd_frame_period % 10) >= 5 ? 1 : 0);
  return microseconds;
}

void PA_write(uint8_t bank, uint8_t reg, uint8_t data)
{
  chip_select(true);
  write(0xef, bank);
  write(reg, data);
  chip_select(false);
}

uint8_t PA_read(uint8_t bank, uint8_t reg)
{
  chip_select(true);
  write(0xef, bank);
  uint8_t value = read(reg);
  chip_select(false);
  return value;
}

void PA_read_report(uint8_t buffer[], int format)
{
  int num_bytes = 256;
  uint8_t format_code = 5;
  
  switch (format)
  {
    default:
      break;
    case 1: // format 1: 256-byte
      num_bytes = 256;
      format_code = 5;
      break;
    case 2: // format 2: 96-byte
      num_bytes = 96;
      format_code = 9;
      break;
    case 3: // format 3: 144-byte
      num_bytes = 144;
      format_code = 10;
      break;
    case 4: // format 4: 208-byte
      num_bytes = 208;
      format_code = 11;
      break;
  }

  chip_select(true);
  write(0xef, format_code);
  burst_read(0, buffer, num_bytes);
  chip_select(false);
}

void PA_read_report(PA_object objs[16], int format)
{
  uint8_t buffer[256];
  PA_read_report(buffer, format);
  for (size_t i = 0; i < 16; i++)
  {
    objs[i].load(&buffer[i*16], format);
  }
}

void PA_init()
{
  // Configure SPI and chip select pin
  pinMode(PIN_CSB, OUTPUT);
  chip_select(false);
  SPI.begin();
  SPI.beginTransaction(SPISettings(14000000, LSBFIRST, SPI_MODE3));

  // Set up PixArt PAJ7025R2
  chip_select(true);
  load_initial_settings();
  chip_select(false);
  // TODO: set frame rate, sensor gain, etc.
  // TODO: learn how frames are actually triggered?
  s_frame_period_micros = PA_get_frame_period_microseconds();
}

void PA_deinit()
{
  SPI.end();
  chip_select(false);
}
