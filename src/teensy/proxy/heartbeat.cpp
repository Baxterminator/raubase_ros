#include "teensy/proxy/heartbeat.hpp"

#include <rclcpp/logging.hpp>
#include <robotbot_msgs/msg/detail/heart_beat_state__struct.hpp>

namespace raubase::teensy::proxy {

void HeartBeatProxy::setup(rclcpp::Node::SharedPtr node) {
  RCLCPP_INFO(logger, "Initializing proxy %s", NODE_NAME);

  // Declaring parameters for the encoder
  refresh_rate = node->declare_parameter("hbt_ms", 500);

  // Initializing working components
  publisher = node->create_publisher<HeartBeatState>(PUBLISHING_TOPIC, QOS);
  clock = node->get_clock();

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

  _msg.stamp = clock->now();

  // Identification
  _msg.device_id = 0;
  _msg.hw_rev = 0;
  _msg.sw_rev = 0;

  // Actual state
  _msg.voltage = 5;
  _msg.load = 3;
  _msg.state = 2;
  _msg.m_right = 1;
  _msg.m_left = 1;

  publisher->publish(_msg);
}

}  // namespace raubase::teensy::proxy