#ifndef INCLUDED_PERSPECTIVE_WINDOW_HPP
#define INCLUDED_PERSPECTIVE_WINDOW_HPP

#include "apps/object_visualizer/window.hpp"
#include "apps/object_visualizer/render.hpp"
#include <opencv2/opencv.hpp>

class perspective_window: public window_3d
{
public:
  perspective_window(int width, int height);
  void init(const pixart::settings &settings) override;
  void update(const std::array<PA_object, 16> &objs) override;

private:
  pixart::settings m_settings;
  cv::Mat m_camera_intrinsic;
  static constexpr float k_object_width = 8e-2f;
  static constexpr float k_object_height = 3e-2f;
  const std::vector<cv::Point3f> m_object_points;

  struct led_position
  {
    int x = 0;
    int y = 0;
    int idx = -1;
  };

  std::array<led_position, 4> m_leds;

  float m_distance = 0;
  float m_ya = 0;
  
  static int distance(const led_position &led1, const PA_object &led2);
  static int distance(const led_position &led1, const led_position &led2);
  static int find_shortest_distance(const std::array<led_position, 4> &leds);

  void sort_by_distance_from(int base_idx, std::array<int, 3> &neighbors_out, const std::array<PA_object, 16> &objs);
  void identify_leds(const std::array<PA_object, 16> &objs);
  size_t match_to_prior(const std::array<PA_object, 16> &leds, int distance_threshold);
  void canonicalize_leds(const std::array<PA_object, 16> &leds);
  void perspective_update(const std::array<PA_object, 16> &objs);
  void draw_led_board(render::vector3 position, render::euler3 rotation);
  void draw_test_scene();
};

#endif  // INCLUDED_PERSPECTIVE_WINDOW_HPP
