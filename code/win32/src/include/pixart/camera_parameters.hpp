#pragma once
#ifndef INCLUDED_PIXART_CAMERA_PARAMETERS_HPP
#define INCLUDED_PIXART_CAMERA_PARAMETERS_HPP

#include <cmath>

namespace pixart
{

  namespace camera_parameters
  {

    static const constexpr double effective_focal_length = 1.484307e-3; // meters
    static const constexpr double fno = 1.838227e-3;                    // meters
    static const constexpr double image_circle = 1.658e-3;              // meters
    static const constexpr double back_focal_length = 0.975e-3;         // meters
    static const constexpr double image_area = 1.524e-3;                // meters (seems to refer to diagonal length of sensor)
    static const constexpr double pixels_x = 98;                        // pixels
    static const constexpr double pixels_y = 98;                        // pixels
    static const constexpr double pixel_width = 11e-6;                  // meters
    static const constexpr double pixel_height = 11e-6;                 // meters

    static double focal_length_x_pixels(double image_pixels_x = pixels_x)
    {
      double image_width = pixel_width * pixels_x;
      return effective_focal_length * image_pixels_x / image_width;
    }

    static double focal_length_y_pixels(double image_pixels_y = pixels_y)
    {
      double image_height = pixel_height * pixels_y;
      return effective_focal_length * image_pixels_y / image_height;
    }

  } // camera_parameters

} // pixart

#endif  // INCLUDED_PIXART_CAMERA_PARAMETERS_HPP
