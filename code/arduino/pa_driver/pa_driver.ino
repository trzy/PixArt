#include "pixart.hpp"
#include "packets.hpp"
#include "cooperative_task.hpp"

static cooperative_task<util::millisecond::resolution> s_blink_led;
static cooperative_task<util::millisecond::resolution> s_read_objects;

static void blink_led(util::time::duration<util::microsecond::resolution> delta, size_t count)
{
  static const bool sequence[] = { true, true, false, false, true, true, false, false, true, false, true, false, false, false, false, false };
  bool on = sequence[count % (sizeof(sequence) / sizeof(bool))];
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}

static void read_objects(util::time::duration<util::microsecond::resolution> delta, size_t count)
{
  PA_object objs[16];
  PA_read_report(objs, 1);
  /*
  for (size_t i = 0; i < 16; i++)
  {
    Serial.print("Object ");
    Serial.print(i, DEC);
    Serial.print("\n");
    Serial.print("--------");
    Serial.print(i >= 10 ? "-\n" : "\n");
    objs[i].print();
  }
  */
  char buffer[1024];
  char *ptr = buffer;
  ptr += sprintf(ptr, "(%d,%d)\n", objs[0].boundary_left, objs[0].boundary_up);
  Serial.print(buffer);
}

void setup()
{
  Serial.begin(9600); 
  s_blink_led = cooperative_task<util::millisecond::resolution>(util::milliseconds(100), blink_led);
  s_read_objects = cooperative_task<util::millisecond::resolution>(util::seconds(1), read_objects);
  PA_init();
}

void loop()
{
  s_read_objects.tick();
  s_blink_led.tick();
}
