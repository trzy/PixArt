#ifndef INCLUDED_RENDER_HPP
#define INCLUDED_RENDER_HPP

namespace cv
{
  class Mat;
}

namespace render
{

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

  void set_camera(float fov_x, float aspect, vector3 position, euler3 rotation);

  namespace node
  {

    struct transform
    {
      transform(vector3 position, vector3 scale, euler3 rotation);
      transform(const cv::Mat &rotation3x3, const cv::Mat &translation3x1);
      ~transform();
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

    struct box
    {
      box(vector3 position, vector3 scale, euler3 rotation, color3 color);
      ~box();
    };

    struct cylinder
    {
      cylinder(vector3 position, float radius, float height, euler3 rotation, color3 color);
      ~cylinder();
    };

  } // node

} // render

#endif  // INCLUDED_RENDER_HPP