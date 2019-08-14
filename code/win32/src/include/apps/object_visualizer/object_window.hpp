#ifndef INCLUDED_OBJECT_WINDOW_HPP
#define INCLUDED_OBJECT_WINDOW_HPP

#include "apps/object_visualizer/window.hpp"
#include "util/config.hpp"
#include <memory>

namespace object_window
{
  static constexpr const char *k_enabled = "ObjectViewWindow/Enabled";
  static constexpr const char *k_resolution = "ObjectViewWindow/Resolution";

  std::shared_ptr<i_window> create(const util::config::Node &config);
}

#endif  // INCLUDED_OBJECT_WINDOW_HPP
