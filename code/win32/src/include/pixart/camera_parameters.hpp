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
    static const constexpr double image_area = 1.524e-6;                // meters^2
    static const constexpr double pixels_x = 98;                        // pixels
    static const constexpr double pixels_y = 98;                        // pixels

    static constexpr double focal_length_x_pixels()
    {
      double image_width = std::sqrt(image_area);
      return effective_focal_length * pixels_x / image_width;
    }

    static constexpr double focal_length_y_pixels()
    {
      double image_height = std::sqrt(image_area);
      return effective_focal_length * pixels_y / image_height;
    }

  } // camera_parameters

} // pixart

#endif  // INCLUDED_PIXART_CAMERA_PARAMETERS_HPP
