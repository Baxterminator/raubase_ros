#include "teensy/proxy/encoder.hpp"

#include <rclcpp/logging.hpp>
#include <sstream>

#include "teensy/interface/message.hpp"

namespace raubase::teensy::proxy {

void EncoderProxy::setupParams(rclcpp::Node::SharedPtr node) {
  RCLCPP_INFO(logger, "Initializing proxy %s", NODE_NAME);

  // Declaring parameters for the encoder
  _refresh_rate = node->declare_parameter("enc_ms", 25);
  _reverse_enc = node->declare_parameter("enc_rev", true);

  // Initializing working components
  publisher = node->create_publisher<EncoderState>(PUBLISHING_TOPIC, QOS);
  clock = node->get_clock();
}

void EncoderProxy::setupSubscriptions() {
  RCLCPP_INFO(logger, "Initializing proxy %s", NODE_NAME);
  subscribeTeensyComponent(TEENSY_COMP, _refresh_rate);
  configEncoderReversing();
}

void EncoderProxy::configEncoderReversing() {
  std::stringstream ss;
  ss << REVERSED_ENC << " " << ((_reverse_enc) ? "1" : "0");
  sendToTeensy(MSG::make(ss.str().c_str()), false);
}

void EncoderProxy::decode(char* msg) {
  // like: enc 16.2 16.5
  /* enc 1 : motor 1 encoding
   *     2 : motor 2 encoding
   */
  if (strlen(msg) <= 4) return;
  const char* p1 = msg + 4;
  _msg.stamp = clock->now();
  _msg.right = -strtoll(p1, (char**)&p1, 10);
  _msg.left = strtoll(p1, (char**)&p1, 10);

  publisher->publish(_msg);
}

}  // namespace raubase::teensy::proxy