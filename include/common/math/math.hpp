#ifndef RAUBASE_MATH
#define RAUBASE_MATH

/*
Copyright (C) 2017-2024 by DTU
Authors:
  Geoffrey Côte: geoffrey.cote@centraliens-nantes.org

The MIT License (MIT)  https://mit-license.org/

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the “Software”), to deal in the Software without
restriction, including without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies
or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <algorithm>
#include <builtin_interfaces/msg/time.hpp>
#include <cmath>

using mTime = builtin_interfaces::msg::Time;

namespace raubase::math {

constexpr const double M_2PI{2 * M_PI};

/**
 * @brief Bounds the input "value" into the interval [-bound, bound].
 * The function is linear in this interval and constant outside:
 *    IF value < -bound => RETURN -bound
 *    IF value >  bound => RETURN bound
 *    ELSE RETURN value
 *
 * @tparam T the type of the value (int, float, double, ...)
 * @param value the value to bound
 * @param bound the maximum (and min) value allowed
 */
template <typename T>
inline T saturate(T value, T bound) {
  return std::max(std::min(value, bound), -bound);
}

/**
 * @brief This function is linear in the interval [-xlim, xlim] and constant outside.
 * The mathematical description is:
 *      f(x) = { value    if value in [-xlim, xlim]
 *             { outside  else
 * @tparam T the type of the value (int, float, double, ...)
 * @param value the value to bound
 * @param x_lim the limit of the interval
 * @param outside the value inside of interval
 */
template <typename T>
inline T cst_outside(T value, T x_lim, T outside = 0) {
  return (-x_lim < value && value < x_lim) ? value : outside;
}

/**
 * @brief Return the natural angle (i.e. the equivalent angle in the interval [-pi, pi]) of the
 * given angle.
 *
 * @tparam T the type of the value (int, float, double, ...)
 * @param value the angle we want to apply this formula to
 */
template <typename T>
inline T natural_angle(T value) {
  return (value > M_PI) ? value - M_2PI : ((value < -M_PI) ? value + M_2PI : value);
}

/**
 * @brief Compute the time between two timestamp of ROS2 messages.
 *
 * @param from the time stamp from the last message
 * @param to  the time stamp of the newer message
 * @return the difference of time in seconds
 */
inline double timeBtwStamps(const mTime &from, const mTime &to) {
  return (to.sec - from.sec + (to.nanosec - from.nanosec) * 1E-9);
}

}  // namespace raubase::math

#endif