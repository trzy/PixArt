/*
 * object_visualizer:
 *
 * Show objects as detected by sensor.
 */

#include "util/logging.hpp"
#include "util/command_line.hpp"
#include "serial/serial_port.hpp"
#include "arduino/packet_reader.hpp"
#include "pa_driver/packets.hpp"
#include "pa_driver/pixart_object.hpp"
#include "apps/object_visualizer/window.hpp"
#include "apps/object_visualizer/print_sensor_settings.hpp"
#include "apps/object_visualizer/print_objects.hpp"
#include <SDL2/SDL.h>
#include <cstdio>
#include <chrono>
#include <memory>
#include <map>

static void render_frames(serial_port *port, const std::shared_ptr<i_window> &window)
{
  const struct
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

  float scale_x = window->width() / 98;
  float scale_y = window->height() / 98;

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

        // Clear window
        window->clear();

        // Decode objects and draw objects
        PA_object objs[16];
        for (int i = 0; i < 16; i++)
        {
          objs[i].load(&response->data[i * 16], response->format);
          SDL_Rect rect;
          rect.x = int(scale_x * objs[i].boundary_left);
          rect.y = int(scale_y * objs[i].boundary_up);
          rect.w = int(scale_x * (objs[i].boundary_right - objs[i].boundary_left));
          rect.h = int(scale_y * (objs[i].boundary_down - objs[i].boundary_up));
          window->draw_rectangle(rect, colors[i].r, colors[i].g, colors[i].b);
        }

        window->update();
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

int main(int argc, char **argv)
{
  util::config::Node config("Global");
  constexpr const char *k_port = "Arduino/SerialPort/PortName";
  constexpr const char *k_baud = "Arduino/SerialPort/BaudRate";
  constexpr const char *k_print_settings = "SettingsPrintout/Enabled";
  constexpr const char *k_view_objs = "ObjectViewWindow/Enabled";
  constexpr const char *k_res2d = "ObjectViewWindow/Resolution";
  constexpr const char *k_print_objs = "ObjectASCIIPrintout/Enabled";

  {
    using namespace util::command_line;
    std::vector<option_definition> options
    {
      switch_option({{ "--help" }}, {{ "-?", "-h", "-help" }}, "ShowHelp", "Print this help text."),
      default_valued_option("--port", string("name"), "COM3", k_port, "Serial port to connect on."),
      default_valued_option("--baud", integer("rate", 300, 115200), "115200", k_baud, "Baud rate."),
      default_valued_option("--settings", util::command_line::boolean(), "true", k_print_settings, "Print PixArt sensor settings."),
      default_valued_option("--view-objects", util::command_line::boolean(), "true", k_view_objs, "Schematic view of detected objects in sensor frame."),
      default_multivalued_option("--res-2d", { integer("width"), integer("height") }, "392,392", k_res2d, "Resolution of 2D object view window."),
      switch_option({ "--print-objects" }, k_print_objs, "Print objects for single frame.")
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

    std::shared_ptr<i_window> obj_window;
    if (config[k_view_objs].ValueAs<bool>())
    {
      obj_window = std::make_shared<window_3d>("PixArt Object View", config[k_res2d]["width"].ValueAs<int>(), config[k_res2d]["height"].ValueAs<int>());
    }

    serial_port arduino_port(config[k_port].Value<std::string>(), config[k_baud].ValueAs<unsigned>());

    if (config[k_print_settings].ValueAs<bool>())
    {
      print_sensor_settings(&arduino_port);
    }

    if (config[k_print_objs].ValueAs<bool>())
    {
      print_objects(&arduino_port);
    }

    if (obj_window)
    {
      render_frames(&arduino_port, obj_window);
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