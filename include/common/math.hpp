#ifndef RAUBASE_MATH
#define RAUBASE_MATH

#include <algorithm>
#include <cmath>

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

template <typename T>
inline T natural_angle(T value) {
  return (value > M_PI) ? value - M_2PI : ((value < -M_PI) ? value + M_2PI : value);
}

}  // namespace raubase::math

#endif