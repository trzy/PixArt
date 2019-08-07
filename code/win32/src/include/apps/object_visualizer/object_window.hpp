#ifndef INCLUDED_OBJECT_WINDOW_HPP
#define INCLUDED_OBJECT_WINDOW_HPP

#include "apps/object_visualizer/window.hpp"

class object_window: public window_3d
{
public:
  object_window(int width, int height);
  void update(const std::array<PA_object, 16> &objs) override;
};

#endif  // INCLUDED_OBJECT_WINDOW_HPP
