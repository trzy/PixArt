#ifndef INCLUDED_PERSPECTIVE_WINDOW_HPP
#define INCLUDED_PERSPECTIVE_WINDOW_HPP

#include "apps/object_visualizer/window.hpp"
#include <memory>

std::shared_ptr<i_window> create_perspective_window(int width, int height);

#endif  // INCLUDED_PERSPECTIVE_WINDOW_HPP
