#include "teensy/proxy/motor.hpp"

#include <cstdio>

#include "teensy/interface/message.hpp"

namespace raubase::teensy::proxy {

void MotorProxy::setupParams(rclcpp::Node::SharedPtr node) {
  RCLCPP_INFO(logger, "Initializing proxy %s", NODE_NAME);

  // Declaring parameters for the encoder
  _max_voltage = node->declare_parameter("mot_max_v", 10);

  // Initializing working components
  subscriber = node->create_subscription<CmdMotorVoltage>(
      SUBSCRIBING_TOPIC, QOS,
      [this](const CmdMotorVoltage::SharedPtr msg) { this->sendCmd(msg, false); });
  clock = node->get_clock();
}

void MotorProxy::setupSubscriptions() { RCLCPP_INFO(logger, "Initializing proxy %s", NODE_NAME); }

void MotorProxy::sendCmd(const CmdMotorVoltage::SharedPtr msg, bool direct) {
  char teensy_msg[MSG::MBL];
  snprintf(teensy_msg, MSG::MBL, MOTOR_CMD, std::min(msg->right, _max_voltage),
           std::min(msg->left, _max_voltage));
  sendToTeensy(MSG::make(teensy_msg, false), direct);
}

void MotorProxy::closeTeensy() {
  RCLCPP_INFO(logger, "Cleaning-up ...");
  auto msg = std::make_shared<CmdMotorVoltage>();
  msg->left = 0;
  msg->right = 0;
  sendCmd(msg, true);
}

}  // namespace raubase::teensy::proxy