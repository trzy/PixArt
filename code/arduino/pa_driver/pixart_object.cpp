#include "pixart_object.hpp"
#include <algorithm>
#include <cstring>

void PA_object::render_ascii(char *output, int pitch, char symbol) const
{
  int lx = std::min((uint8_t) 97, boundary_left);
  int rx = std::min((uint8_t) 97, boundary_right);
  int uy = std::min((uint8_t) 97, boundary_up);
  int dy = std::min((uint8_t) 97, boundary_down);
  for (int y = uy; y <= dy; y++)
  {
    for (int x = lx; x <= rx; x++)
    {
      output[y * pitch + x] = symbol;
    }
  }
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
