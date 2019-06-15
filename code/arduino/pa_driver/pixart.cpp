#include "pixart.hpp"
#include <Arduino.h>
#include <SPI.h>

static constexpr uint8_t PIN_CSB = A0;
static unsigned s_frame_period_micros = 0;

void PA_object::render(char *output, int pitch, char symbol)
{
  int lx = min(97, boundary_left);
  int rx = min(97, boundary_right);
  int uy = min(97, boundary_up);
  int dy = min(97, boundary_down);
  //Serial.print(lx,DEC);Serial.print(",");Serial.print(rx,DEC);Serial.print(",");Serial.print(uy,DEC);Serial.print(",");Serial.print(dy,DEC);Serial.print("\n");
  for (int y = uy; y <= dy; y++)
  {
    for (int x = lx; x <= rx; x++)
    {
      output[y * pitch + x] = symbol;
    }
  }
}

void PA_object::print()
{
  char buffer[1024];
  char *ptr = buffer;
  ptr += sprintf(ptr, "center          = (%d,%d)\n", cx, cy);
  ptr += sprintf(ptr, "area            = %d\n", area);
  ptr += sprintf(ptr, "avg. brightness = %d\n", average_brightness);
  ptr += sprintf(ptr, "max brightness  = %d\n", max_brightness);
  ptr += sprintf(ptr, "range           = %d\n", range);
  ptr += sprintf(ptr, "radius          = %d\n", radius);
  ptr += sprintf(ptr, "boundary        = (%d,%d,%d,%d)\n", boundary_left, boundary_right, boundary_up, boundary_down);
  ptr += sprintf(ptr, "aspect          = %d\n", aspect_ratio);
  ptr += sprintf(ptr, "vx              = %d\n", vx);
  ptr += sprintf(ptr, "vy              = %d\n", vy);
  Serial.print(buffer);
}

void PA_object::load(const uint8_t *data, int format)
{
  memset(this, 0, sizeof(this));

  // Formats 1-4
  area = data[0] | ((data[1] & 0x3f) << 8);
  cx = data[2] | ((data[3] & 0x0f) << 8);
  cy = data[4] | ((data[5] & 0x0f) << 8);

  // Format 1, 3
  if (format == 1 || format == 3)
  {
    average_brightness = data[6];
    max_brightness = data[7];
    range = data[8] >> 4;
    radius = data[8] & 0xf;
  }

  if (format == 1 || format == 4)
  {
    int offset = format == 4 ? 3 : 0;
    boundary_left = data[9 - offset] & 0x7f;
    boundary_right = data[10 - offset] & 0x7f;
    boundary_up = data[11 - offset] & 0x7f;
    boundary_down = data[12 - offset] & 0x7f;
    aspect_ratio = data[13 - offset];
    vx = data[14 - offset];
    vy = data[15 - offset];
  }
}

PA_object::PA_object(const uint8_t *data, int format)
{
  load(data, format);
}

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

double PA_get_frame_period_microseconds()
{
  chip_select(true);
  write(0xef, 0x0c); // bank C
  uint32_t cmd_frame_period = read(0x07);
  cmd_frame_period |= read(0x08) << 8;
  cmd_frame_period |= read(0x09) << 16;
  chip_select(false);
  double frame_period_100ns = (double) cmd_frame_period;
  double frame_period_millis = frame_period_100ns * 1e-1; // 100 ns -> us
  return frame_period_millis;
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
  s_frame_period_micros = (unsigned) PA_get_frame_period_microseconds();
}

void PA_deinit()
{
  SPI.end();
  chip_select(false);
}
