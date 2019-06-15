#ifndef INCLUDED_PIXART_HPP
#define INCLUDED_PIXART_HPP

#include <cstdint>

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

  void render(char *output, int pitch, char symbol);
  void print();
  void load(const uint8_t *data, int format);
  PA_object(const uint8_t *data, int format);
  PA_object()
  {
  }
};

double PA_get_frame_period_microseconds();
void PA_write(uint8_t bank, uint8_t reg, uint8_t data);
uint8_t PA_read(uint8_t bank, uint8_t reg);
void PA_read_report(uint8_t buffer[], int format);
void PA_read_report(PA_object objs[16], int format);
void PA_init();
void PA_deinit();

#endif  // INCLUDED_PIXART_HPP
