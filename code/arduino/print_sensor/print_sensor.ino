/*
 * Reads object data from PixArt sensor using Adafruit nRF52832 Feather.
 * PCB board connector J1 leads:
 *
 *  +--------------+
 *  |    PixArt    |
 *  |              |
 *  | C2        C1 |
 *  |              |
 *  | J1           |
 *  | -----------  |
 *  +-| | | | | |--+
 *    | | | | | |
 *    | | | | | |
 *    | | | | | |
 *    | | | | | GND
 *    | | | | G12/MOSI
 *    | | | |
 *    | | | G11/MISO
 *    | | |
 *    | | G10/SCK
 *    | |
 *    | G9/CSB
 *    |
 *    VDDMA
 *
 * Connections to nRF52832 board headers:
 *
 *    VDDMA    -> +3.3v (recommend pin nearest GND)
 *    G9/CSB   -> A0
 *    G10/SCK  -> SCK
 *    G11/MISO -> MISO (note MISO and MOSI ordering reversed from PixArt PCB)
 *    G12/MOSI -> MOSI
 *    GND      -> GND
 *
 * Vishay Semiconductors TSHA4400 IR LED (875nm) successfully detectable
 * by the PixArt sensor.
 */

#include <cstdio>
#include <cstring>
#include <SPI.h>

static unsigned s_frame_period_micros;

struct PA_object
{
  uint16_t area;
  uint16_t cx;
  uint16_t cy;
  uint8_t average_brightness;
  uint8_t max_brightness;
  uint8_t range;
  uint8_t radius;
  uint8_t boundary_left;
  uint8_t boundary_right;
  uint8_t boundary_up;
  uint8_t boundary_down;
  uint8_t aspect_ratio;
  uint8_t vx;
  uint8_t vy;

  void render(char *output, int pitch, char symbol)
  {
    int lx = min(97, boundary_left);
    int rx = min(97, boundary_right);
    int uy = min(97, boundary_up);
    int dy = min(97, boundary_down);
    Serial.print(lx,DEC);Serial.print(",");Serial.print(rx,DEC);Serial.print(",");Serial.print(uy,DEC);Serial.print(",");Serial.print(dy,DEC);Serial.print("\n");
    for (int y = uy; y <= dy; y++)
    {
      for (int x = lx; x <= rx; x++)
      {
        output[y * pitch + x] = symbol;
      }
    }
  }

  void print()
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

  void load(const uint8_t *data, int format)
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

  PA_object(const uint8_t *data, int format)
  {
    load(data, format);
  }

  PA_object()
  {
  }
};

void PA_write(uint8_t reg, uint8_t data)
{
  SPI.transfer(0x00);     // bit 7 = write (0), bits 6-0 = single byte (0)
  SPI.transfer(reg);
  SPI.transfer(data);
}

uint8_t PA_read(uint8_t reg)
{
  SPI.transfer(0x80);     // bit 7 = read (1), bits 6-0 = single byte (0)
  SPI.transfer(reg);
  return SPI.transfer(0);
}

void PA_burst_read(uint8_t reg_base, uint8_t buffer[], uint16_t num_bytes)
{
  SPI.transfer(0x81);
  SPI.transfer(reg_base);
  for (uint16_t i = 0; i < num_bytes; i++)
  {
    buffer[i] = SPI.transfer(0);
  }
}

void PA_print_settings()
{
    // Register bank 0
    PA_write(0xef, 0);
    uint16_t product_id = (uint16_t(PA_read(0x03)) << 8) | PA_read(0x02);
    uint16_t dsp_area_max_threshold = (PA_read(0x0c) << 8) | PA_read(0x0b);
    uint8_t dsp_noise_threshold = PA_read(0x0f);
    uint8_t dsp_orientation_ratio = PA_read(0x10);
    uint8_t dsp_orientation_factor = PA_read(0x11);
    uint8_t dsp_max_object_num = PA_read(0x19);

    // Register bank 1
    PA_write(0xef, 1);
    uint8_t sensor_gain_1 = PA_read(0x05);
    uint8_t sensor_gain_2 = PA_read(0x06);
    uint16_t sensor_exposure_length = (PA_read(0x0f) << 8) | PA_read(0x0e);

    // Register bank C
    PA_write(0xef, 0x0c);
    uint16_t cmd_scale_resolution_x = (PA_read(0x61) << 8) | PA_read(0x60);
    uint16_t cmd_scale_resolution_y = (PA_read(0x63) << 8) | PA_read(0x62);

    // Reset back to bank 0
    PA_write(0xef, 0);

    // Print everything
    char buffer[2048];
    char *ptr = buffer;
    ptr += sprintf(ptr, "PAJ7025R2 Settings\n");
    ptr += sprintf(ptr, "------------------\n");
    ptr += sprintf(ptr, "Product ID                    = %04x\n", product_id);
    ptr += sprintf(ptr, "DSP area max threshold        = %04x\n", dsp_area_max_threshold);
    ptr += sprintf(ptr, "DSP noise threshold           = %02x\n", dsp_noise_threshold);
    ptr += sprintf(ptr, "DSP orientation ratio         = %02x\n", dsp_orientation_ratio);
    ptr += sprintf(ptr, "DSP orientation factor        = %02x\n", dsp_orientation_factor);
    ptr += sprintf(ptr, "DSP maximum number of objects = %02x\n", dsp_max_object_num);
    ptr += sprintf(ptr, "Sensor gain 1                 = %02x\n", sensor_gain_1);
    ptr += sprintf(ptr, "Sensor gain 2                 = %02x\n", sensor_gain_2);
    ptr += sprintf(ptr, "Sensor exposure length        = %04x\n", sensor_exposure_length);
    ptr += sprintf(ptr, "Scale resolution X            = %04x\n", cmd_scale_resolution_x);
    ptr += sprintf(ptr, "Scale resolution Y            = %04x\n", cmd_scale_resolution_y);
    ptr += sprintf(ptr, "Frame period                  = %1.4f ms\n", PA_get_frame_period_milliseconds());
    Serial.print(buffer);
}

void PA_load_initial_settings()
{
  PA_write(0xef, 0);
  PA_write(0xdc, 0);
  PA_write(0xfb, 4);
  PA_write(0xef, 0);
  PA_write(0x2f, 5);
  PA_write(0x30, 0);
  PA_write(0x30, 1);
  PA_write(0x1f, 0);
  PA_write(0xef, 1);
  PA_write(0x2d, 0);
  PA_write(0xef, 0xc);
  PA_write(0x64, 0);
  PA_write(0x65, 0);
  PA_write(0x66, 0);
  PA_write(0x67, 0);
  PA_write(0x68, 0);
  PA_write(0x69, 0);
  PA_write(0x6a, 0);
  PA_write(0x6b, 0);
  PA_write(0x6c, 0);
  PA_write(0x71, 0);
  PA_write(0x72, 0);
  PA_write(0x12, 0);
  PA_write(0x13, 0);
  PA_write(0xef, 0);
  PA_write(0x01, 1);
}

void PA_set_frame_rate(double hz)
{
  double period = 1.0 / hz;
  double period_100ns = 1e7 * period;
  uint32_t cmd_frame_period = (uint32_t) round(period_100ns);
  PA_write(0xef, 0x0c); // bank C
  PA_write(0x07, cmd_frame_period & 0xff);
  PA_write(0x08, (cmd_frame_period >> 8) & 0xff);
  PA_write(0x09, (cmd_frame_period >> 16) & 0xf);

  char buffer[2048];
  sprintf(buffer, "cmd_frame_period = 0x%05x\n", cmd_frame_period);
  Serial.print(buffer);
}

void PA_set_sensor_exposure_time(double time)
{
  double exposure_time_200ns = time / 200e-9;
  uint32_t b_expo = (unsigned int) round(exposure_time_200ns);
  PA_write(0xef, 0x0c); // bank C
  PA_write(0x0f, b_expo & 0xff);
  PA_write(0x10, (b_expo >> 8) & 0xff);
  PA_write(0xef, 0x01); // APPLY_COMMAND_1
  PA_write(0x01, 0x01); // APPLY_COMMAND_2

  char buffer[2048];
  sprintf(buffer, "b_expo = 0x%04x\n", b_expo);
  Serial.print(buffer);
}

void PA_set_sensor_gain(uint8_t b_global, uint8_t b_ggh)
{
  PA_write(0xef, 0x0c); // bank C
  PA_write(0x0b, b_global & 0x1f);
  PA_write(0x0c, b_ggh & 3);
  PA_write(0xef, 0x01); // APPLY_COMMAND_1
  PA_write(0x01, 0x01); // APPLY_COMMAND_2
}

void PA_set_debug_image(uint8_t image_num)
{
  PA_write(0xef, 0x01); // bank 1
  PA_write(0x2b, image_num);
  //PA_write(0xef, 0x01); // APPLY_COMMAND_1 (needed?)
  PA_write(0x01, 0x01); // APPLY_COMMAND_2 (...)
}

double PA_get_frame_period_microseconds()
{
  PA_write(0xef, 0x0c); // bank C
  uint32_t cmd_frame_period = PA_read(0x07);
  cmd_frame_period |= PA_read(0x08) << 8;
  cmd_frame_period |= PA_read(0x09) << 16;
  double frame_period_100ns = (double) cmd_frame_period;
  double frame_period_millis = frame_period_100ns * 1e-1; // 100 ns -> us
  return frame_period_millis;
}

double PA_get_frame_period_milliseconds()
{
  return 1e-3 * PA_get_frame_period_microseconds();
}

void PA_read_report(PA_object objs[16], int format)
{
  uint8_t report[256];
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

  PA_write(0xef, format_code);
  PA_burst_read(0, report, num_bytes);

  int label_size = num_bytes / 16;
  for (int i = 0; i < 16; i++)
  {
    objs[i].load(&report[i * label_size], format);
  }
}

void PA_init()
{

  // SPI begin
  SPI.beginTransaction(SPISettings(14000000, LSBFIRST, SPI_MODE3));

  // Set up
  digitalWrite(A0, 0);  // chip select
  PA_load_initial_settings();
  //PA_set_frame_rate(30);
  //PA_set_frame_rate(200.88);
  //PA_set_sensor_exposure_time(1.6384e-3);
  //PA_set_sensor_gain(0x10, 0);
  PA_set_debug_image(0);
  PA_print_settings();
  s_frame_period_micros = (unsigned) PA_get_frame_period_microseconds();

  digitalWrite(A0, 1);

  // Read first frame
  for (int i = 0; i < 1; i++)
  {
    delayMicroseconds(s_frame_period_micros);
  }
  digitalWrite(A0, 0);
  PA_object objs[16];
  PA_read_report(objs, 1);
  digitalWrite(A0, 1);

  // SPI end
  //SPI.endTransaction();

  // Print detected objects
  for (int i = 0; i < 16; i++)
  {
    Serial.print("Object ");
    Serial.print(i, DEC);
    Serial.print("\n");
    Serial.print("--------");
    Serial.print(i >= 10 ? "-\n" : "\n");
    objs[i].print();
  }

  // Render image
  char *image = (char *) malloc((98 + 1) * 98 + 1);
  memset(image, '.', 99 * 98);
  for (int y = 0; y < 98; y++)
  {
    image[y * 99 + 98] = '\n';
  }
  image[99 * 98] = 0;

  for (int i = 0; i < 16; i++)
  {
    char symbol = i < 10 ? ('0' + i) : ('a' + i - 10);
    objs[i].render(image, 99, symbol);
  }
  Serial.print("Image:\n");
  Serial.print(image);
  free(image);
}

void setup() {
  Serial.begin(9600);

  pinMode(A0, OUTPUT);
  digitalWrite(A0, 1);
  SPI.begin();

  PA_init();


  //SPI.end();

  //digitalWrite(A0, 1);
  //SPI.beginTransaction(SPISettings(14000000, LSBFIRST, SPI_MODE3));
}

void loop() {

  // put your main code here, to run repeatedly
  delayMicroseconds(s_frame_period_micros);
  digitalWrite(A0, 0);
  PA_object objs[16];
  PA_read_report(objs, 1);
  digitalWrite(A0, 1);  // deasserting CS seems to be required for next frame readout

  char buffer[1024];
  char *ptr = buffer;
  ptr += sprintf(ptr, "(%d,%d)\n", objs[0].boundary_left, objs[0].boundary_up);
  Serial.print(buffer);

  delayMicroseconds(1000000);

}
