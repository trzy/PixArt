/*
 * object_visualizer:
 *
 * Show objects as detected by sensor.
 */

#include "pa_driver/packets.hpp"
#include "pa_driver/pixart_object.hpp"
#include "arduino/packet_reader.hpp"
#include "serial/serial_port.hpp"
#include "util/logging.hpp"
#include "util/command_line.hpp"
#include <SDL2/SDL.h>
#include <cstdio>
#include <chrono>
#include <map>

static double compute_frame_period_seconds(uint32_t frame_period_reg)
{
  double period_100ns = (double) frame_period_reg;
  double period = period_100ns * 100e-9f;
  return period;
}

static void print_sensor_settings(serial_port *port)
{
  struct
  {
    uint8_t bank;
    uint8_t reg;
  } registers[] =
  {
    { 0x00, 0x02 }, // product ID
    { 0x00, 0x03 }, //
    { 0x00, 0x0f }, // DSP noise threshold
    { 0x00, 0x0b }, // DSP max area threshold
    { 0x00, 0x0c }, //
    { 0x00, 0x10 }, // DSP orientation ratio
    { 0x00, 0x11 }, // DSP orientation factor
    { 0x00, 0x19 }, // DSP maximum object number
    { 0x01, 0x05 }, // sensor gain 1
    { 0x01, 0x06 }, // sensor gain 2
    { 0x01, 0x0e }, // sensor exposure length
    { 0x01, 0x0f }, //
    { 0x0c, 0x60 }, // interpolated resolution x
    { 0x0c, 0x61 }, //
    { 0x0c, 0x62 }, // interpolated resolution y
    { 0x0c, 0x63 }, //
    { 0x0c, 0x07 }, // frame period
    { 0x0c, 0x08 }, //
    { 0x0c, 0x09 }  //
  };

  // Responses are indexed with a key comprising bank and address
  std::map<uint16_t, uint8_t> values;
  packet_reader reader(
    [&](uint8_t *buffer, size_t size) -> size_t
    {
      return port->read(buffer, size);
    },
    [&](PacketID id, const uint8_t *buffer, size_t size) -> bool
    {
      if (id == PacketID::PeekResponse)
      {
        const peek_response_packet *response = reinterpret_cast<const peek_response_packet *>(buffer);
        uint16_t key = (response->bank << 8) | response->address;
        values.emplace(key, response->data);
        return true;
      }
      return false;
    }
  );

  // Send peek request for each register
  size_t num_requests = sizeof(registers) / sizeof(registers[0]);
  for (size_t i = 0; i <num_requests; i++)
  {
    peek_packet peek(registers[i].bank, registers[i].reg);
    port->write(peek);
  }

  // Wait for all responses
  reader.wait_for_packets(num_requests);

  // Decode registers
  uint16_t product_id = (values[0x0003] << 8) | values[0x0002];
  uint16_t max_area_threshold = (values[0x000c] << 8) | values[0x000b];
  uint8_t noise_threshold = values[0x000f];
  uint8_t orientation_ratio = values[0x0010];
  uint8_t orientation_factor = values[0x0011];
  uint8_t max_object_number = values[0x0019];
  uint8_t sensor_gain_1 = values[0x0105];
  uint8_t sensor_gain_2 = values[0x0106];
  uint16_t sensor_exposure_length = (values[0x010f] << 8) | values[0x010e];
  uint16_t interpolated_resolution_x = (values[0x0c61] << 8) | values[0x0c60];
  uint16_t interpolated_resolution_y = (values[0x0c63] << 8) | values[0x0c62];
  uint32_t frame_period_reg = (values[0x0c09] << 16) | (values[0x0c08] << 8) | values[0x0c07];
  double frame_period = compute_frame_period_seconds(frame_period_reg);
  double frame_rate = 1.0f / frame_period;

  // Print
  printf("PAJ7025R2 Settings\n");
  printf("------------------\n");
  printf("Product ID                    = 0x%04x %s\n", product_id, product_id == 0x7025 ? "" : "(unknown device)");
  printf("DSP area max threshold        = 0x%04x\n", max_area_threshold);
  printf("DSP noise threshold           = 0x%02x\n", noise_threshold);
  printf("DSP orientation ratio         = 0x%02x\n", orientation_ratio);
  printf("DSP orientation factor        = 0x%02x\n", orientation_factor);
  printf("DSP maximum number of objects = %d\n", max_object_number);
  printf("Sensor gain 1                 = 0x%02x\n", sensor_gain_1);
  printf("Sensor gain 2                 = 0x%02x\n", sensor_gain_2);
  printf("Sensor exposure length        = 0x%04x\n", sensor_exposure_length);
  printf("Scale resolution X            = %d\n", interpolated_resolution_x);
  printf("Scale resolution Y            = %d\n", interpolated_resolution_y);
  printf("Frame period                  = %1.4f ms\n", frame_period * 1000);
  printf("Frame rate                    = %1.2f Hz\n", frame_rate);
  printf("\n");
}

static void render_ascii_image(const PA_object objs[16])
{
  // Clear buffer
  char image[98 + 1][98 + 1];
  memset(image, '.', sizeof(image));
  for (int y = 0; y < 98; y++)
  {
    image[y][98] = '\n';
  }
  image[98][0] = 0;

  // Draw objects into buffer
  for (int i = 0; i < 16; i++)
  {
    char symbol = i < 10 ? ('0' + i) : ('a' + i - 10);
    objs[i].render_ascii((char *) image, 98 + 1, symbol);
  }

  printf("Sensor Frame\n");
  printf("------------\n");
  printf(&image[0][0]);
  printf("\n");
}

static void print_objects(const PA_object objs[16])
{
  for (int i = 0; i < 16; i++)
  {
    auto &obj = objs[i];
    printf("Object %d\n", i);
    printf("--------%c\n", i >= 10 ? '-' : 0);
    printf("center          = (%d,%d)\n", obj.cx, obj.cy);
    printf("area            = %d\n", obj.area);
    printf("avg. brightness = %d\n", obj.average_brightness);
    printf("max brightness  = %d\n", obj.max_brightness);
    printf("range           = %d\n", obj.range);
    printf("radius          = %d\n", obj.radius);
    printf("boundary        = (%d,%d,%d,%d)\n", obj.boundary_left, obj.boundary_right, obj.boundary_up, obj.boundary_down);
    printf("aspect          = %d\n", obj.aspect_ratio);
    printf("vx              = %d\n", obj.vx);
    printf("vy              = %d\n", obj.vy);
    printf("\n");
  }
}

static void render_frame_ascii(serial_port *port)
{
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

        // Decode objects
        PA_object objs[16];
        for (int i = 0; i < 16; i++)
        {
          objs[i].load(&response->data[i * 16], response->format);
        }

        // Draw them and print object information
        render_ascii_image(objs);
        print_objects(objs);
        return true;
      }
      return false;
    }
  );

  object_report_request_packet request;
  port->write(request);
  reader.wait_for_packets(1);
}

static void render_frames(serial_port *port, SDL_Window *window)
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

  int width;
  int height;
  SDL_GetWindowSize(window, &width, &height);
  float scale_x = width / 98;
  float scale_y = height / 98;

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
        SDL_Surface *surface = SDL_GetWindowSurface(window);
        SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 0, 0, 0));

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
          SDL_FillRect(surface, &rect, SDL_MapRGB(surface->format, colors[i].r, colors[i].g, colors[i].b));
        }
        SDL_UpdateWindowSurface(window);

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
  //reader.wait_for_packets(1);
}

int main(int argc, char **argv)
{
  util::config::Node config("Global");
  constexpr const char *k_port = "Arduino/SerialPort/PortName";
  constexpr const char *k_baud = "Arduino/SerialPort/BaudRate";
  constexpr const char *k_res2d = "SensorFrameWindow/Resolution";

  {
    using namespace util::command_line;
    std::vector<option_definition> options
    {
      switch_option({{ "--help" }}, {{ "-?", "-h", "-help" }}, "ShowHelp", "Print this help text."),
      default_valued_option("--port", string("name"), "COM3", k_port, "Serial port to connect on."),
      default_valued_option("--baud", integer("rate", 300, 115200), "115200", k_baud, "Baud rate."),
      default_multivalued_option("--res-2d", { integer("width"), integer("height") }, "392,392", k_res2d, "Resolution of 2D object view window.")
    };
    auto state = parse_command_line(&config, options, argc, argv);
    if (state.exit)
    {
      return state.parse_error ? 1 : 0;
    }
  }

  try
  {
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
      LOG_ERROR("SDL failed to initialize: " << SDL_GetError());
      return 1;
    }

    SDL_Window *obj_window = SDL_CreateWindow(
      "PixArt Object View",
      SDL_WINDOWPOS_UNDEFINED,
      SDL_WINDOWPOS_UNDEFINED,
      config[k_res2d]["width"].ValueAs<int>(),
      config[k_res2d]["height"].ValueAs<int>(),
      SDL_WINDOW_SHOWN);

    if (obj_window == nullptr)
    {
      LOG_ERROR("Failed to create object view window: " << SDL_GetError());
    }

    serial_port arduino_port(config[k_port].Value<std::string>(), config[k_baud].ValueAs<unsigned>());
    print_sensor_settings(&arduino_port);
    render_frames(&arduino_port, obj_window);

    SDL_DestroyWindow(obj_window);
    SDL_Quit();
  }
  catch (std::exception& e)
  {
    LOG_ERROR("Exception caught: " << e.what());
    return 1;
  }

  return 0;
}