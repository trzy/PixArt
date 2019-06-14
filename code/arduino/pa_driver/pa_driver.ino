#include "pixart.hpp"
#include "packets.hpp"

void setup()
{
  Serial.begin(9600);
  PA_init();
}

void loop()
{
  PA_object objs[16];
  delay(1000);
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
