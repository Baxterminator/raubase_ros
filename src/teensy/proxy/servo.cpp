#include "teensy/proxy/servo.hpp"

#include <functional>

#include "teensy/interface/proxy_interface.hpp"

namespace raubase::teensy::proxy {

void ServoProxy::setupParams(rclcpp::Node::SharedPtr node) {
  RCLCPP_INFO(logger, "Initializing proxy %s", NODE_NAME);

  // Declaring parameters for the encoder
  _is_on = node->declare_parameter(Params::PROXY_ON, Default::PROXY_ON);
  _n_servos = node->declare_parameter(Params::N_SERVOS, Default::N_SERVOS);
  _refresh_rate = node->declare_parameter(Params::RATE_MS, Default::RATE_MS);

  // Initializing working components
  _servo_states.enabled.resize(_n_servos);
  _servo_states.positons.resize(_n_servos);
  _servo_states.velocities.resize(_n_servos);
  cmd_sub = node->create_subscription<CmdServoPosition>(
      Topics::CMD_SUB, QOS, std::bind(&ServoProxy::setServoPosition, this, std::placeholders::_1));
  clock = node->get_clock();
}

void ServoProxy::setupSubscriptions() {
  RCLCPP_INFO(logger, "Initializing proxy %s", NODE_NAME);
  subscribeTeensyComponent(TEENSY_COMP, _refresh_rate);
}

void ServoProxy::setServoPosition(const CmdServoPosition::SharedPtr msg) {
  char teensy_msg[MSG::MBL];
  snprintf(teensy_msg, MSG::MBL, SERVO_MSG, msg->servo_id, msg->position, msg->velocity);
  sendToTeensy(MSG::make(teensy_msg, false), false);
}

void ServoProxy::decode(char* msg) {
  if (!_is_on) return;

  // like: servo {enable position velocity} x 5
  if (strlen(msg) <= 5) return;
  const char* p1 = msg + 5;
  _servo_states.stamp = clock->now();

  for (int i = 0; i < _n_servos; i++) {
    _servo_states.enabled[i] = strtoll(p1, (char**)&p1, 10);
    _servo_states.positons[i] = strtoll(p1, (char**)&p1, 10);
    _servo_states.velocities[i] = strtoll(p1, (char**)&p1, 10);
  }

  state_pub->publish(_servo_states);
}

}  // namespace raubase::teensy::proxy