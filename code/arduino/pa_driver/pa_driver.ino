/*
 * pa_driver.ino:
 *
 * Main program for Adafruit nRF52832 Feather board.
 *
 * PixArt PCB board connector J1 leads:
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
 */

#include "pixart.hpp"
#include "packets.hpp"
#include "cooperative_task.hpp"

static util::cooperative_task<util::millisecond::resolution> s_led_blinker;
static util::cooperative_task<util::microsecond::resolution> s_frame_reader;
static bool s_send_object_report = false;

static void blink_led(util::time::duration<util::microsecond::resolution> delta, size_t count)
{
  static const bool sequence[] = { true, true, false, false, true, true, false, false, true, false, true, false, false, false, false, false };
  bool on = sequence[count % (sizeof(sequence) / sizeof(bool))];
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}

static void read_frame(util::time::duration<util::microsecond::resolution> delta, size_t count)
{
  object_report_packet report(1);
  PA_read_report(report.data, 1);
  if (s_send_object_report)
  {
    Serial.write(reinterpret_cast<const uint8_t *>(&report), sizeof(report));
    s_send_object_report = false;
  }

  /*
  PA_object objs[16];
  PA_read_report(objs, 1);
  for (size_t i = 0; i < 16; i++)
  {
    Serial.print("Object ");
    Serial.print(i, DEC);
    Serial.print("\n");
    Serial.print("--------");
    Serial.print(i >= 10 ? "-\n" : "\n");
    objs[i].print();
  }
  char buffer[1024];
  char *ptr = buffer;
  ptr += sprintf(ptr, "(%d,%d)\n", objs[0].boundary_left, objs[0].boundary_up);
  Serial.print(buffer);
  */
}

static void process_packet(const uint8_t *buffer)
{
  const packet_header *header = reinterpret_cast<const packet_header *>(buffer);
  switch (header->id)
  {
  default:
    break;
  case PacketID::Poke:
  {
    const poke_packet *poke = reinterpret_cast<const poke_packet *>(buffer);
    PA_write(poke->bank, poke->address, poke->data);
    break;
  }
  case PacketID::Peek:
  {
    const peek_packet *peek = reinterpret_cast<const peek_packet *>(buffer);
    peek_response_packet peek_response(peek->bank, peek->address, PA_read(peek->bank, peek->address));
    Serial.write(reinterpret_cast<const uint8_t *>(&peek_response), sizeof(peek_response));
    break;
  }
  case PacketID::ObjectReportRequest:
  {
    s_send_object_report = true;
    break;
  }
  }
}

//TODO: serial buffer only holds 64 bytes!
static void read_serial_port()
{
  static uint8_t s_packet_buffer[255 * 2];
  if (Serial.available() > 0)
  {
    int packet_bytes = Serial.peek() * 2;
    if (Serial.available() >= packet_bytes)
    {
      Serial.readBytes(s_packet_buffer, packet_bytes);
      process_packet(s_packet_buffer);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  PA_init();
  uint32_t frame_period = PA_get_frame_period_microseconds();
  s_led_blinker = util::cooperative_task<util::millisecond::resolution>(util::milliseconds(100), blink_led);
  s_frame_reader = util::cooperative_task<util::microsecond::resolution>(util::microseconds(frame_period), read_frame);
}

void loop()
{
  s_frame_reader.tick();
  s_led_blinker.tick();
  read_serial_port();
}
