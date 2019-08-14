#include "apps/object_visualizer/object_window.hpp"

class object_window_impl: public window_3d
{
public:
  object_window_impl(int width, int height)
    : window_3d("PixArt Object View", width, height)
  {
  }

  void update(const std::array<PA_object, 16> &objs)
  {
    static const struct
    {
      uint8_t r;
      uint8_t g;
      uint8_t b;
    } colors[] =
    {
      { 0xff, 0x00, 0x00 },
      { 0x00, 0xff, 0x00 },
      { 0x00, 0x00, 0xff },
      { 0x00, 0xff, 0xff },
      { 0xff, 0x00, 0xff },
      { 0xff, 0xff, 0x00 },
      { 0xff, 0xff, 0xff },
      { 0xff, 0x80, 0x00 },

      { 0x7f, 0x00, 0x00 },
      { 0x00, 0x7f, 0x00 },
      { 0x00, 0x00, 0x7f },
      { 0x00, 0x7f, 0x7f },
      { 0x7f, 0x00, 0x7f },
      { 0x7f, 0x7f, 0x00 },
      { 0x7f, 0x7f, 0x7f },
      { 0x7f, 0x40, 0x00 }
    };

    float scale_x = width() / 98;
    float scale_y = height() / 98;

    clear();

    for (size_t i = 0; i < objs.size(); i++)
    {
      SDL_Rect rect;
      rect.x = int(scale_x * objs[i].boundary_left);
      rect.y = int(scale_y * objs[i].boundary_up);
      rect.w = int(scale_x * (objs[i].boundary_right - objs[i].boundary_left));
      rect.h = int(scale_y * (objs[i].boundary_down - objs[i].boundary_up));
      draw_rectangle(rect, colors[i].r, colors[i].g, colors[i].b);
    }
  }
};

namespace object_window
{
  std::shared_ptr<i_window> create(const util::config::Node &config)
  {
    int width = config[k_resolution]["width"].ValueAs<int>();
    int height = config[k_resolution]["height"].ValueAs<int>();
    return std::make_shared<object_window_impl>(width, height);
  }
}
