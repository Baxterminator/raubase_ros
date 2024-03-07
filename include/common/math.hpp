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
 * @return T
 */
template <typename T>
inline T saturate(T value, T bound) {
  return std::max(std::min(value, bound), -bound);
}

template <typename T>
inline T natural_angle(T value) {
  return (value > M_PI) ? value - M_2PI : ((value < M_PI) ? value + M_2PI : value);
}

}  // namespace raubase::math

#endif