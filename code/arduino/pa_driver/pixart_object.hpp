#pragma once
#ifndef INCLUDED_PIXART_OBJECT_HPP
#define INCLUDED_PIXART_OBJECT_HPP

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

  void render_ascii(char *output, int pitch, char symbol) const;
  void load(const uint8_t *data, int format);
  PA_object(const uint8_t *data, int format);
  PA_object()
  {
  }
};

#endif  // INCLUDED_PIXART_OBJECT_HPP
