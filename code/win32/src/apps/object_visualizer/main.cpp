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
#include "apps/object_visualizer/window.hpp"
#include "apps/object_visualizer/print_sensor_settings.hpp"
#include "apps/object_visualizer/print_objects.hpp"
#include <SDL2/SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <cstdio>
#include <chrono>
#include <memory>
#include <map>
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

struct vector3
{
  float x = 0;
  float y = 0;
  float z = 0;

  static vector3 zero()
  {
    return vector3(0, 0, 0);
  }

  static vector3 one()
  {
    return vector3(1, 1, 1);
  }

  static vector3 right()
  {
    return vector3(1, 0, 0);
  }

  static vector3 up()
  {
    return vector3(0, 1, 0);
  }

  static vector3 forward()
  {
    return vector3(0, 0, -1);
  }

  vector3(float in_x, float in_y, float in_z)
    : x(in_x),
      y(in_y),
      z(in_z)
  {
  }

  vector3()
  {
  }

  vector3 operator+(const vector3 &rhs) const
  {
    return vector3(x + rhs.x, y + rhs.y, z + rhs.z);
  }

  vector3 operator*(float scalar) const
  {
    return vector3(x * scalar, y * scalar, z * scalar);
  }
};

struct euler3: public vector3
{
  static euler3 zero()
  {
    return euler3(0, 0, 0);
  }

  static euler3 right()
  {
    return euler3(1, 0, 0);
  }

  static euler3 up()
  {
    return euler3(0, 1, 0);
  }

  static euler3 forward()
  {
    return euler3(0, 0, -1);
  }

  euler3(float x, float y, float z)
    : vector3(x, y, z)
  {
  }

  euler3 operator*(float scalar) const
  {
    return euler3(x * scalar, y * scalar, z * scalar);
  }
};

struct color3
{
  float r;
  float g;
  float b;

  static color3 black()
  {
    return color3(0, 0, 0);
  }

  static color3 white()
  {
    return color3(1, 1, 1);
  }

  static color3 gray()
  {
    return color3(0.5f, 0.5f, 0.5f);
  }

  static color3 red()
  {
    return color3(1, 0, 0);
  }

  static color3 green()
  {
    return color3(0, 1, 0);
  }

  static color3 blue()
  {
    return color3(0, 0, 1);
  }

  color3(float in_r, float in_g, float in_b)
    : r(in_r),
      g(in_g),
      b(in_b)
  {
  }
};

namespace node
{
  struct transform
  {
    transform(vector3 position, vector3 scale, euler3 rotation)
    {
      glPushMatrix();
      glTranslatef(position.x, position.y, position.z);
      glRotatef(rotation.z, 0, 0, 1);
      glRotatef(rotation.y, 0, 1, 0);
      glRotatef(rotation.x, 1, 0, 0);
      glScalef(scale.x, scale.y, scale.z);
    }

    ~transform()
    {
      glPopMatrix();
    }
  };

  struct translate: public transform
  {
    translate(vector3 position)
      : transform(position, vector3::one(), euler3::zero())
    {
    }
  };

  struct scale: public transform
  {
    scale(vector3 dimensions)
      : transform(vector3::zero(), dimensions, euler3::zero())
    {
    }
  };

  struct rotate: public transform
  {
    rotate(euler3 rotation)
      : transform(vector3::zero(), vector3::one(), rotation)
    {
    }
  };

  class box
  {
  public:
    box(vector3 position, vector3 scale, euler3 rotation, color3 color)
    {
      draw(position, scale, rotation, color);
    }

    ~box()
    {
      glPopMatrix();
    }

  private:
    void draw(vector3 position, vector3 scale, euler3 rotation, color3 color)
    {
      glPushMatrix();
      glTranslatef(position.x, position.y, position.z);
      glRotatef(rotation.z, 0, 0, 1);
      glRotatef(rotation.y, 0, 1, 0);
      glRotatef(rotation.x, 1, 0, 0);
      glScalef(scale.x, scale.y, scale.z);
      glBegin(GL_QUADS);
      glColor3f(color.r, color.g, color.b);
      glVertex3f(-0.5f, 0.5f, 0.5f); glVertex3f(0.5f, 0.5f, 0.5f); glVertex3f(0.5f, -0.5f, 0.5f); glVertex3f(-0.5f, -0.5f, 0.5f);     // front
      glVertex3f(0.5f, 0.5f, -0.5f); glVertex3f(-0.5f, 0.5f, -0.5f); glVertex3f(-0.5f, -0.5f, -0.5f); glVertex3f(0.5f, -0.5f, -0.5f); // back
      glVertex3f(-0.5f, 0.5f, -0.5f); glVertex3f(0.5f, 0.5f, -0.5f); glVertex3f(0.5f, 0.5f, 0.5f); glVertex3f(-0.5f, 0.5f, 0.5f);     // top
      glVertex3f(-0.5f, -0.5f, 0.5f); glVertex3f(0.5f, -0.5f, 0.5f); glVertex3f(0.5f, -0.5f, -0.5f); glVertex3f(-0.5f, -0.5f, -0.5f); // bottom
      glVertex3f(-0.5f, 0.5f, -0.5f); glVertex3f(-0.5f, 0.5f, 0.5f); glVertex3f(-0.5f, -0.5f, 0.5f); glVertex3f(-0.5f, -0.5f, -0.5f); // left
      glVertex3f(0.5f, 0.5f, 0.5f); glVertex3f(0.5f, 0.5f, -0.5f); glVertex3f(0.5f, -0.5f, -0.5f); glVertex3f(0.5f, -0.5f, 0.5f);     // right
      glEnd();
    }
  };
}

class perspective_window: public window_3d
{
public:
  perspective_window(int width, int height)
    : window_3d("Perspective View", width, height)
  {
  }

  void update(const std::array<PA_object, 16> &objs) override
  {
    clear();
    perspective(60);
    camera(vector3(0, -1.5, 0), vector3(30, 0, 0));
    {



      {
        node::transform transform(vector3::forward() * m_distance, vector3::one(), euler3::up() * m_ya);

        // Render target
        {

          color3 ir_color = color3(0.7f, 0, 0.9f);    // purple
          color3 power_color = color3(0.9f, 0.8f, 0); // amber

          // Breadboard
          { node::box box(vector3::zero(), vector3(1, 0.5f, 0.01f), euler3::zero(), color3(0, 0.5f, 0)); }

          // IR LEDs
          { node::box box(vector3(-0.45f, 0.20f, 0.05f), vector3(0.05f, 0.05f, 0.1f), euler3::zero(), ir_color); }
          { node::box box(vector3(0.45f, 0.20f, 0.05f), vector3(0.05f, 0.05f, 0.1f), euler3::zero(), ir_color); }
          { node::box box(vector3(-0.45f, -0.20f, 0.05f), vector3(0.05f, 0.05f, 0.1f), euler3::zero(), ir_color); }
          { node::box box(vector3(0.45f, -0.20f, 0.05f), vector3(0.05f, 0.05f, 0.1f), euler3::zero(), ir_color); }

          // Power LED
          { node::box box(vector3(0, 0.20f, 0.05f), vector3(0.05f, 0.05f, 0.1f), euler3::zero(), power_color); }
        }
      }
    }
    //box(vector3(0, 0, -m_distance), vector3(0, m_ya, 0), 1);
    m_distance += .01f;
    m_ya += 1;
  }

private:
  float m_distance = 0;
  float m_ya = 0;

  static constexpr float pi()
  {
    return (float) std::atan(1) * 4;
  }

  static float deg2rad(float angle)
  {
    return angle * pi() / 180;
  }

  void perspective(float fov)
  {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = float(width()) / float(height());
    float fov_y = fov * aspect;
    gluPerspective(fov_y, (GLfloat) aspect, 0.1f, 1e2f);
  }

  void camera(vector3 position, vector3 euler)
  {
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef(-euler.z, 0, 0, 1);
    glRotatef(-euler.y, 0, 1, 0);
    glRotatef(-euler.x, 1, 0, 0);
    glTranslatef(-position.x, -position.y, -position.z);
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

static void render_frames(i_serial_device *port, const std::vector<std::shared_ptr<i_window>> &windows)
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
        for (auto &window: windows)
        {
          window->update(objs);
        }

        for (auto &window: windows)
        {
          window->blit();
        }

        return true;
      }
      return false;
    }
  );

  port->write(request);
  bool quit = false;
  SDL_Event e;
  while (!quit)
  {
    reader.tick();

    while (SDL_PollEvent(&e) != 0)
    {
      if (e.type == SDL_QUIT)
      {
        quit = true;
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

    std::vector<std::shared_ptr<i_window>> windows;

    if (config[k_view_objs].ValueAs<bool>())
    {
      auto window = std::make_shared<object_window>(config[k_res2d]["width"].ValueAs<int>(), config[k_res2d]["height"].ValueAs<int>());
      windows.push_back(window);
    }

    if (config[k_view_3d].ValueAs<bool>())
    {
      auto window = std::make_shared<perspective_window>(config[k_res3d]["width"].ValueAs<int>(), config[k_res3d]["height"].ValueAs<int>());
      windows.push_back(window);
    }

    std::shared_ptr<i_serial_device> arduino_port = create_serial_connection(config);

    if (config[k_print_settings].ValueAs<bool>())
    {
      print_sensor_settings(arduino_port.get());
    }

    if (config[k_print_objs].ValueAs<bool>())
    {
      print_objects(arduino_port.get());
    }

    if (windows.size() > 0)
    {
      render_frames(arduino_port.get(), windows);
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