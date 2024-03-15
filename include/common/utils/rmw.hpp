#ifndef RAUBASE_RMW
#define RAUBASE_RMW

#include <rclcpp/qos.hpp>
namespace raubase {

inline rclcpp::QoS make_transient() {
  auto qos = rclcpp::QoS(1);
  qos.transient_local();
  return qos;
}

const auto TRANSIENT_QOS = make_transient();

}  // namespace raubase

#endif