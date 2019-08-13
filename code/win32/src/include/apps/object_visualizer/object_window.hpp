#ifndef INCLUDED_OBJECT_WINDOW_HPP
#define INCLUDED_OBJECT_WINDOW_HPP

#include "apps/object_visualizer/window.hpp"
#include <memory>

std::shared_ptr<i_window> create_object_window(int width, int height);

#endif  // INCLUDED_OBJECT_WINDOW_HPP
