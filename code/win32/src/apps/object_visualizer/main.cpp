/*
 * object_visualizer:
 *
 * Show objects as detected by sensor.
 */

#include "util/logging.hpp"
#include "util/command_line.hpp"
#include "serial/serial_port.hpp"
#include "serial/serial_replay_device.hpp"
#include "arduino/packet_reader.hpp"
#include "pa_driver/packets.hpp"
#include "pa_driver/pixart_object.hpp"
#include "pixart/camera_parameters.hpp"
#include "apps/object_visualizer/window.hpp"
#include "apps/object_visualizer/render.hpp"
#include "apps/object_visualizer/sensor_settings.hpp"
#include "apps/object_visualizer/print_objects.hpp"
#include <opencv2/opencv.hpp>
#include <SDL2/SDL.h>
#include <cstdio>
#include <chrono>
#include <memory>
#include <set>
#include <cmath>

static constexpr const char *k_port = "Arduino/SerialPort/PortName";
static constexpr const char *k_baud = "Arduino/SerialPort/BaudRate";
static constexpr const char *k_record_to = "Arduino/SerialPort/Record";
static constexpr const char *k_replay_from = "Arduino/SerialPort/Replay";
static constexpr const char *k_print_settings = "SettingsPrintout/Enabled";
static constexpr const char *k_print_objs = "ObjectASCIIPrintout/Enabled";
static constexpr const char *k_view_objs = "ObjectViewWindow/Enabled";
static constexpr const char *k_res2d = "ObjectViewWindow/Resolution";
static constexpr const char *k_view_3d = "PerspectiveViewWindow/Enabled";
static constexpr const char *k_res3d = "PerspectiveViewWindow/Resolution";

class perspective_window: public window_3d
{
public:
  perspective_window(int width, int height)
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
  }

  void init(const pixart::settings &settings) override
  {
    float fx = pixart::camera_parameters::focal_length_x_pixels(settings.resolution_x);
    float fy = pixart::camera_parameters::focal_length_y_pixels(settings.resolution_y);
    float cx = 0.5f * settings.resolution_x;
    float cy = 0.5f * settings.resolution_y;
    m_camera_intrinsic = (cv::Mat_<float>(3, 3) <<
      fx, 0,  cx,
      0,  fy, cy,
      0,  0,  1);
    std::cout << "fx="<<fx<<std::endl;
  }

  void update(const std::array<PA_object, 16> &objs) override
  {
    identify_leds(objs);
    perspective_update(objs);
    //draw_test_scene();
    //std::cout << objs[0].cx << ',' << objs[0].cy << std::endl;
  }

private:
  struct led_position
  {
    int x = 0;
    int y = 0;
    int idx = -1;
  };

  cv::Mat m_camera_intrinsic;
  std::array<led_position, 4> m_leds;
  float m_distance = 0;
  float m_ya = 0;

  static constexpr float k_object_width = 8e-2f;
  static constexpr float k_object_height = 3e-2f;
  const std::vector<cv::Point3f> m_object_points;

  static int distance(const PA_object &led1, const PA_object &led2)
  {
    int dx = led1.cx - led2.cx;
    int dy = led1.cy - led2.cy;
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
        return distance(objs[base_idx], objs[idx1]) < distance(objs[base_idx], objs[idx2]);
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

  void perspective_update(const std::array<PA_object, 16> &objs)
  {
    // Image points from sensor
    std::vector<cv::Point2f> image_points;
    image_points.reserve(m_object_points.size());
    std::cout <<"--"<<std::endl;
    for (auto &led: m_leds)
    {
      image_points.emplace_back(float(led.x), float(led.y));
      std::cout<<led.x<<","<<led.y<<std::endl;
    }
    //std::cout << objs[0].cx << ',' << objs[0].cy << std::endl;

    // Solve for model-view transform from image points
    cv::Mat rodrigues;
    cv::Mat translation;
    bool result = cv::solvePnPRansac(m_object_points, image_points, m_camera_intrinsic, cv::Mat(), rodrigues, translation, false); //, cv::SOLVEPNP_ITERATIVE);

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

class object_window: public window_3d
{
public:
  object_window(int width, int height)
    : window_3d("PixArt Object View", width, height)
  {
  }

  void update(const std::array<PA_object, 16> &objs) override
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

static void remove_window(std::set<std::shared_ptr<i_window>> *windows, SDL_Window *sdl_window)
{
  for (auto it = windows->begin(); it != windows->end(); )
  {
    auto window = *it;
    if (window->window() == sdl_window)
    {
      it = windows->erase(it);
    }
    else
    {
      ++it;
    }
  }
}

static void render_frames(i_serial_device *port, const pixart::settings &settings, std::set<std::shared_ptr<i_window>> *windows)
{
  object_report_request_packet request;

  packet_reader reader(
    [&](uint8_t *buffer, size_t size) -> size_t
    {
      return port->read(buffer, size);
    },
    [&](PacketID id, const uint8_t *buffer, size_t size) -> bool
    {
      if (id == PacketID::ObjectReport)
      {
        const object_report_packet *response = reinterpret_cast<const object_report_packet *>(buffer);

        // Request next
        port->write(request);

        // Decode objects
        std::array<PA_object, 16> objs;
        for (int i = 0; i < 16; i++)
        {
          objs[i].load(&response->data[i * 16], response->format);
        }

        // Update views
        for (auto &window: *windows)
        {
          window->update(objs);
        }

        for (auto &window: *windows)
        {
          window->blit();
        }

        return true;
      }
      return false;
    }
  );

  // Initialize windows
  for (auto &window: *windows)
  {
    window->init(settings);
  }

  // Start rendering frames
  port->write(request);
  bool quit = false;
  SDL_Event e;
  while (!quit)
  {
    reader.tick();

    while (SDL_PollEvent(&e) != 0)
    {
      switch (e.type)
      {
      case SDL_QUIT:
        quit = true;
        break;
      case SDL_WINDOWEVENT:
        if (e.window.event == SDL_WINDOWEVENT_CLOSE)
        {
          remove_window(windows, SDL_GetWindowFromID(e.window.windowID));
        }
        break;
      case SDL_MOUSEWHEEL:
        break;
      }
    }
  }
}

static std::shared_ptr<i_serial_device> create_serial_connection(const util::config::Node &config)
{
  const std::string port_name = config[k_port].Value<std::string>();
  const unsigned baud = config[k_baud].ValueAs<unsigned>();

  bool record = config[k_record_to].Exists();
  bool replay = config[k_replay_from].Exists();

  if (record && replay)
  {
    throw std::logic_error("Record and replay options are mutually exclusive");
  }
  else if (replay)
  {
    std::string file = config[k_replay_from].ValueAs<std::string>();
    auto replayer = std::make_shared<serial_replay_device>(file);
    LOG_INFO("Replaying from '" << file << "'...\n");
    return replayer;
  }
  else if (record)
  {
    std::string file = config[k_record_to].ValueAs<std::string>();
    std::unique_ptr<serial_port> port = std::make_unique<serial_port>(port_name, baud);
    std::shared_ptr<serial_replay_device> recorder = std::make_shared<serial_replay_device>(file, std::move(port));
    LOG_INFO("Recording " << port_name << " to '" << file << "'...\n");
    return recorder;
  }

  return std::make_shared<serial_port>(port_name, baud);
}

int main(int argc, char **argv)
{
  util::config::Node config("Global");

  {
    using namespace util::command_line;
    std::vector<option_definition> options
    {
      switch_option({{ "--help" }}, {{ "-?", "-h", "-help" }}, "ShowHelp", "Print this help text."),
      default_valued_option("--port", string("name"), "COM3", k_port, "Serial port to connect on."),
      default_valued_option("--baud", integer("rate", 300, 115200), "115200", k_baud, "Baud rate."),
      valued_option("--record-to", string("file"), k_record_to, "Capture a recording of the serial port data."),
      valued_option("--replay-from", string("file"), k_replay_from, "Replay captured serial port data."),
      default_valued_option("--settings", util::command_line::boolean(), "true", k_print_settings, "Print PixArt sensor settings."),
      switch_option({ "--print-objects" }, k_print_objs, "Print objects for single frame."),
      default_valued_option("--view-objects", util::command_line::boolean(), "true", k_view_objs, "Schematic view of detected objects in sensor frame."),
      default_multivalued_option("--res-2d", { integer("width"), integer("height") }, "392,392", k_res2d, "Resolution of 2D object view window."),
      default_valued_option("--view-3d", util::command_line::boolean(), "true", k_view_3d, "Perspective view of detected objects."),
      default_multivalued_option("--res-3d", { integer("width"), integer("height") }, "640,640", k_res3d, "Resolution of perspective view window.")
    };
    auto state = parse_command_line(&config, options, argc, argv);
    if (state.exit)
    {
      return state.parse_error ? 1 : 0;
    }
  }

  int error = 0;

  try
  {
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
      LOG_ERROR("SDL failed to initialize: " << SDL_GetError());
      return 1;
    }

    std::set<std::shared_ptr<i_window>> windows;

    if (config[k_view_objs].ValueAs<bool>())
    {
      auto window = std::make_shared<object_window>(config[k_res2d]["width"].ValueAs<int>(), config[k_res2d]["height"].ValueAs<int>());
      windows.insert(window);
    }

    if (config[k_view_3d].ValueAs<bool>())
    {
      auto window = std::make_shared<perspective_window>(config[k_res3d]["width"].ValueAs<int>(), config[k_res3d]["height"].ValueAs<int>());
      windows.insert(window);
    }

    std::shared_ptr<i_serial_device> arduino_port = create_serial_connection(config);

    pixart::settings settings = read_sensor_settings(arduino_port.get(), config[k_print_settings].ValueAs<bool>());

    if (config[k_print_objs].ValueAs<bool>())
    {
      print_objects(arduino_port.get());
    }

    if (windows.size() > 0)
    {
      render_frames(arduino_port.get(), settings, &windows);
    }
  }
  catch (std::exception& e)
  {
    LOG_ERROR("Exception caught: " << e.what());
    error = 1;
  }

  SDL_Quit();

  return error;
}