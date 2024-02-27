#include "teensy/proxy/heartbeat.hpp"

#include <rclcpp/logging.hpp>
#include <robotbot_msgs/msg/detail/heart_beat_state__struct.hpp>

namespace raubase::teensy::proxy {

void HeartBeatProxy::setupParams(rclcpp::Node::SharedPtr node) {
  RCLCPP_INFO(logger, "Initializing proxy %s", NODE_NAME);

  // Declaring parameters for the encoder
  refresh_rate = node->declare_parameter("hbt_ms", 500);

  // Initializing working components
  publisher = node->create_publisher<HeartBeatState>(PUBLISHING_TOPIC, QOS);
  clock = node->get_clock();
}

void HeartBeatProxy::setupSubscriptions() {
  RCLCPP_INFO(logger, "Initializing proxy %s", NODE_NAME);
  subscribeTeensyComponent(TEENSY_COMP, refresh_rate);
}

void HeartBeatProxy::decode(char* msg) {
  // like: regbot:hbt 37708.7329 74 1430 5.01 0 6 1 1
  /* hbt 1 : time in seconds, updated every sample time
   *     2 : device ID (probably 1)
   *     3 : software revision number - from SVN * 10 + REV_MINOR
   *     4 : Battery voltage
   *     5 : state
   *     6 : hw type
   *     7 : load
   *     8,9 : motor enabled (left,right)
   */
  if (strlen(msg) < 5) return;
  const char* p1 = msg + 5;

  // Time code, Teensy time in seconds
  strtof64(p1, (char**)&p1);
  _msg.stamp = clock->now();
  //_msg.stamp = 0;

  _msg.device_id = strtol(p1, (char**)&p1, 10);
  _msg.sw_rev = strtol(p1, (char**)&p1, 10);
  _msg.voltage = strtof(p1, (char**)&p1);
  _msg.state = strtol(p1, (char**)&p1, 10);
  _msg.hw_rev = strtol(p1, (char**)&p1, 10);

  _msg.load = strtol(p1, (char**)&p1, 10);
  _msg.m_right = strtol(p1, (char**)&p1, 10);
  _msg.m_left = strtol(p1, (char**)&p1, 10);

  publisher->publish(_msg);
}

}  // namespace raubase::teensy::proxy