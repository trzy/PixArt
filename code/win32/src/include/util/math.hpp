#pragma once
#ifndef INCLUDED_UTIL_MATH_HPP
#define INCLUDED_UTIL_MATH_HPP

#include <cmath>

namespace util
{

  namespace math
  {
    static constexpr const float Pi = atan(1)*4;
    static constexpr const float Rad2Deg = 180.0 / Pi;
    static constexpr const float Deg2Rad = Pi / 180.0;
  }

}

#endif  // INCLUDED_UTIL_MATH_HPP
