#include "apps/object_visualizer/perspective_window.hpp"
#include "apps/object_visualizer/render.hpp"
#include "pixart/camera_parameters.hpp"
#include <opencv2/opencv.hpp>
#include <algorithm>

class perspective_window_impl: public window_3d
{
public:
  perspective_window_impl(int width, int height, const std::string &algo)
    : window_3d("Perspective View", width, height),
      m_object_points
      {
        // Positions of target LEDs in object-local space (world units), in
        // progressive scan order (top to bottom, left to right) as they would
        // appear when facing the camera
        { -0.5f * k_object_width, 0.5f * k_object_height, 0 },  // top left
        { 0.5f * k_object_width, 0.5f * k_object_height, 0 },   // top right corner
        { -0.5f * k_object_width, -0.5f * k_object_height, 0 }, // bottom left corner
        { 0.5f * k_object_width, -0.5f * k_object_height, 0 }   // bottom right corner
      }
  {
    assert(m_leds.size() == m_object_points.size());
    m_pnp_solver = &perspective_window_impl::solve;
    m_solver_algo = cv::SOLVEPNP_ITERATIVE;
  }

  void init(const pixart::settings &settings)
  {
    float fx = pixart::camera_parameters::focal_length_x_pixels(settings.resolution_x);
    float fy = pixart::camera_parameters::focal_length_y_pixels(settings.resolution_y);
    float cx = 0.5f * settings.resolution_x;
    float cy = 0.5f * settings.resolution_y;
    m_camera_intrinsic = (cv::Mat_<float>(3, 3) <<
      fx, 0,  cx,
      0,  fy, cy,
      0,  0,  1);
    //std::cout << "fx="<<fx<<std::endl;
  }

  void update(const std::array<PA_object, 16> &objs)
  {
    canonicalize_leds(objs);
    perspective_update(objs);
    //draw_test_scene();
    //std::cout << objs[0].cx << ',' << objs[0].cy << std::endl;
  }

private:
  typedef bool (perspective_window_impl::*solver_callback)(int algo, const std::vector<cv::Point3f> &object_points, const std::vector<cv::Point2f> &image_points, cv::Mat &rotation, cv::Mat &translation);
  solver_callback m_pnp_solver;
  int m_solver_algo;

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

  static int is_on_screen(const PA_object &led)
  {
    return led.cx < 0xfff && led.cy < 0xfff;
  }

  static int distance(const PA_object &led1, const PA_object &led2)
  {
    int dx = led1.cx - led2.cx;
    int dy = led1.cy - led2.cy;
    return dx*dx + dy*dy;
  }

  static int distance(const led_position &led1, const PA_object &led2)
  {
    int dx = led1.x - led2.cx;
    int dy = led1.y - led2.cy;
    return dx*dx + dy*dy;
  }

  //TODO: PA_object cx,cy -> x,y and template all these
  static int distance(const led_position &led1, const led_position &led2)
  {
    int dx = led1.x - led2.x;
    int dy = led1.y - led2.y;
    return dx*dx + dy*dy;
  }

  //TODO: clean up hard-coded arrays
  struct edge
  {
    int idx1; // top-most (vertical edge) or left-most (horizontal edge)
    int idx2;
    float midpoint_x;
    float midpoint_y;

    void set_vertical(int led1_idx, int led2_idx, const std::array<PA_object, 16> &leds)
    {
      if (leds[led1_idx].cy < leds[led2_idx].cy)
      {
        idx1 = led1_idx;
        idx2 = led2_idx;
      }
      else
      {
        idx1 = led2_idx;
        idx2 = led1_idx;
      }
      midpoint_x = 0.5f * (leds[led1_idx].cx + leds[led2_idx].cx);
      midpoint_y = 0.5f * (leds[led1_idx].cy + leds[led2_idx].cy);
    }

    void set_horizontal(int led1_idx, int led2_idx, const std::array<PA_object, 16> &leds)
    {
      if (leds[led1_idx].cx < leds[led2_idx].cx)
      {
        idx1 = led1_idx;
        idx2 = led2_idx;
      }
      else
      {
        idx1 = led2_idx;
        idx2 = led1_idx;
      }
      midpoint_x = 0.5f * (leds[led1_idx].cx + leds[led2_idx].cx);
      midpoint_y = 0.5f * (leds[led1_idx].cy + leds[led2_idx].cy);
    }
  };


  void sort_by_distance_from(int base_idx, std::array<int, 3> &neighbors_out, const std::array<PA_object, 16> &objs)
  {
    // Neighbors of base
    int j = 0;
    for (int i = 0; i < neighbors_out.size() + 1; i++)
    {
      if (i != base_idx)
      {
        neighbors_out[j++] = i;
      }
    }

    // Sort by distance from base
    std::sort(neighbors_out.begin(), neighbors_out.end(),
      [base_idx, &objs](int idx1, int idx2)
      {
        return perspective_window_impl::distance(objs[base_idx], objs[idx1]) < perspective_window_impl::distance(objs[base_idx], objs[idx2]);
      });
  }

  void identify_leds(const std::array<PA_object, 16> &objs)
  {
    edge vertical[2];
    edge horizontal[2];

    // Identify first vertical and horizontal edges
    std::array<int, 3> neighbors;
    int corner_idx = 0;
    sort_by_distance_from(corner_idx, neighbors, objs);
    vertical[0].set_vertical(corner_idx, neighbors[0], objs); // vertical edge is the shorter one
    horizontal[0].set_horizontal(corner_idx, neighbors[1], objs);

    // The remaining unused corner will form remaining two edges
    corner_idx = neighbors[2];
    sort_by_distance_from(corner_idx, neighbors, objs);
    vertical[1].set_vertical(corner_idx, neighbors[0], objs);
    horizontal[1].set_horizontal(corner_idx, neighbors[1], objs);

    // Sort vertical edges from left to right
    if (vertical[0].midpoint_x > vertical[1].midpoint_x)
    {
      std::swap(vertical[0], vertical[1]);
    }

    //TODO: horizontal edges not needed
    // Now we know which image points correspond to the 4 corners in scan order
    m_leds[0].idx = vertical[0].idx1;
    m_leds[1].idx = vertical[1].idx1;
    m_leds[2].idx = vertical[0].idx2;
    m_leds[3].idx = vertical[1].idx2;
    for (int i = 0; i < 4; i++)
    {
      m_leds[i].x = objs[m_leds[i].idx].cx;
      m_leds[i].y = objs[m_leds[i].idx].cy;
    }
  }

  int find_shortest_distance(const std::array<led_position, 4> &leds)
  {
    int shortest = std::numeric_limits<int>::max();

    for (int i = 0; i < leds.size(); i++)
    {
      if (leds[i].idx < 0)
      {
        // Not assigned
        continue;
      }

      for (int j = i + 1; j < leds.size(); j++)
      {
        if (leds[j].idx < 0)
        {
          continue;
        }
        shortest = std::min(shortest, distance(leds[i], leds[j]));
      }
    }

    return shortest;
  }

  //TODO: rename leds to sensor_observations or something
  size_t match_to_prior(const std::array<PA_object, 16> &leds, int distance_threshold)
  {
    size_t num_matched = 0;

    std::array<bool, 16> used;
    std::fill_n(used.begin(), used.size(), false);

    // For each LED identified in previous frame...
    for (int i = 0; i < m_leds.size(); i++)
    {
      led_position &prev_frame = m_leds[i];
      if (prev_frame.idx < 0)
      {
        // Previous frame position did not exist. Cannot track.
        continue;
      }

      // Greedily find nearest LED in *this* frame that has not yet been used
      int best_idx = -1;
      int best_distance = std::numeric_limits<int>::max();
      for (int j = 0; j < leds.size(); j++)
      {
        if (used[j] || !is_on_screen(leds[j]))
        {
          continue;
        }

        int dist = distance (prev_frame, leds[j]);
        if (dist < best_distance && dist < distance_threshold)
        {
          best_idx = j;
          best_distance = dist;
        }
      }

      // If we found something, update
      if (best_idx >= 0)
      {
        m_leds[i].idx = best_idx;
        m_leds[i].x = leds[best_idx].cx;
        m_leds[i].y = leds[best_idx].cy;
        num_matched += 1;
        used[best_idx] = true;
      }
    }

    //std::cout << "matched="<<num_matched<<std::endl;
    return num_matched;
  }

  void canonicalize_leds(const std::array<PA_object, 16> &leds)
  {
    // Try matching to prior frame. Use half the distance of the vertical edge
    // (which should be the shortest distance between LEDs in sensor frame) as
    // threshold.
    int threshold = find_shortest_distance(m_leds) / 4; // 4 because all distances are square distances
    size_t num_matched = match_to_prior(leds, threshold);

    // If could not match all, perform ab initio identification
    if (num_matched < m_leds.size())
    {
      identify_leds(leds);
    }
  }

  void perspective_update(const std::array<PA_object, 16> &objs)
  {
    // Image points from sensor
    std::vector<cv::Point2f> image_points;
    image_points.reserve(m_object_points.size());
    //std::cout <<"--"<<std::endl;
    for (auto &led: m_leds)
    {
      image_points.emplace_back(float(led.x), float(led.y));
      //std::cout<<led.x<<","<<led.y<<std::endl;
    }
    //std::cout << objs[0].cx << ',' << objs[0].cy << std::endl;

    // Solve for model-view transform from image points
    cv::Mat rodrigues;
    cv::Mat translation;
    bool result = (this->*(m_pnp_solver))(m_solver_algo, m_object_points, image_points, rodrigues, translation);//cv::solvePnPRansac(m_object_points, image_points, m_camera_intrinsic, cv::Mat(), rodrigues, translation, false); //, cv::SOLVEPNP_ITERATIVE);

    // Render
    if (result)
    {
      //auto pos = cv::Point3f(translation);
      //std::cout<<pos << std::endl;

      // Convert Rodrigues vector to 3x3 matrix
      cv::Mat rotation;
      cv::Rodrigues(rodrigues, rotation);

      using namespace render;

      // Set up camera
      clear();
      float horizontal_fov = 45;
      float aspect = float(width()) / float(height());
      set_camera(45, aspect, vector3(0, 0, 0), euler3::zero());

      // Apply modelview transform. Flip Z to convert OpenCV camera -> OpenGL
      // coordinates. Flip Y so that +y is down, as in sensor frame.
      node::scale scale(vector3(1,-1,-1));
      node::transform transform(rotation, translation);

      // Draw board
      draw_led_board(vector3::zero(), euler3::zero());
    }
  }

  bool solve(int algo, const std::vector<cv::Point3f> &object_points, const std::vector<cv::Point2f> &image_points, cv::Mat &rotation, cv::Mat &translation)
  {
    return cv::solvePnP(object_points, image_points, m_camera_intrinsic, cv::Mat(), rotation, translation, false, algo);
  }

  bool solve_ransac(int algo, const std::vector<cv::Point3f> &object_points, const std::vector<cv::Point2f> &image_points, cv::Mat &rotation, cv::Mat &translation)
  {
    int iterations = 100;
    float reprojection_error = 8;
    int min_inliners = 100;
    return cv::solvePnPRansac(object_points, image_points, m_camera_intrinsic, cv::Mat(), rotation, translation, false, iterations, reprojection_error, min_inliners, cv::noArray(), algo);
  }

  void draw_led_board(render::vector3 position, render::euler3 rotation)
  {
    using namespace render;

    float board_width = 8e-2f;
    float board_height = 3e-2f;
    color3 board_color = color3(0, 0.5f, 0);
    color3 ir_color = color3(0.7f, 0, 0.9f);    // purple
    color3 power_color = color3(0.9f, 0.8f, 0); // amber

    // Position and orientation
    node::transform transform(position, vector3::one(), rotation);

    // Breadboard
    node::box board(vector3::zero(), vector3(board_width, board_height, 0.002f), euler3::zero(), board_color);

    // Board components
    {
      float led_x = 0.4f; // % of board dimensions
      float led_y = 0.4f;
      float led_width = 0.05f;
      float led_height = led_width * (board_width / board_height);
      float led_depth = 2.0f;

      // Move pivot point to surface of board
      node::translate surface(vector3(0, 0, 0.5f));

      // IR LEDs
      { node::box led(vector3(-led_x, led_y, 0.5f * led_depth), vector3(led_width, led_height, led_depth), euler3::zero(), ir_color); }
      { node::box led(vector3(led_x, led_y, 0.5f * led_depth), vector3(led_width, led_height, led_depth), euler3::zero(), ir_color); }
      { node::box led(vector3(led_x, -led_y, 0.5f * led_depth), vector3(led_width, led_height, led_depth), euler3::zero(), ir_color); }
      { node::box led(vector3(-led_x, -led_y, 0.5f * led_depth), vector3(led_width, led_height, led_depth), euler3::zero(), ir_color); }

      // Power LEDs
      { node::box led(vector3(0, led_y, 0.5f * led_depth), vector3(led_width, led_height, led_depth), euler3::zero(), power_color); }
    }
  }

  void draw_test_scene()
  {
    clear();

    {
      using namespace render;
      float horizontal_fov = 60;
      float aspect = float(width()) / float(height());
      set_camera(60, aspect, vector3(0, -0.1f, 0), euler3(30, 0, 0));
      draw_led_board(vector3::forward() * m_distance, euler3::up() * m_ya);
    }

    m_distance += .05f * (1 / 60.0f) ;
    m_ya += 1;
  }
};

namespace perspective_window
{
  std::shared_ptr<i_window> create(const util::config::Node &config)
  {
    int width = config[k_resolution]["width"].ValueAs<int>();
    int height = config[k_resolution]["height"].ValueAs<int>();
    return std::make_shared<perspective_window_impl>(width, height, "iterative");
  }
}