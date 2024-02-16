#include "teensy/proxy/encoder.hpp"

#include <rclcpp/logging.hpp>

namespace raubase::teensy::proxy {

void EncoderProxy::setup(rclcpp::Node::SharedPtr node) {
  RCLCPP_INFO(logger, "Initializing proxy %s", NODE_NAME);

  // Declaring parameters for the encoder
  refresh_rate = node->declare_parameter("enc_ms", 8);

  // Initializing working components
  publisher = node->create_publisher<EncoderState>(PUBLISHING_TOPIC, QOS);

  subscribeTeensyComponent(TEENSY_COMP, refresh_rate);
  configEncoderReversing();
}

void EncoderProxy::configEncoderReversing() {}

void EncoderProxy::decode(char*) {}

}  // namespace raubase::teensy::proxy