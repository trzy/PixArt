#include "pa_driver/pixart_object.hpp"
#include "pa_driver/packets.hpp"
#include "arduino/packet_reader.hpp"
#include "serial/i_serial_device.hpp"
#include <cstdio>

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

void print_objects(i_serial_device *port)
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
