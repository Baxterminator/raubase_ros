#include "simulator/plugins/Motor.hpp"

namespace raubase::simu::plugins {

void Motor::setup(rclcpp::Node::SharedPtr node) {
  // ROS
  _clock = node->get_clock();

  // Motor
  _last_motor = std::make_shared<MotorVoltage>();
  _last_motor->right = 0;
  _last_motor->left = 0;
  mot_righ_km = node->declare_parameter("mot_right_km", DEFAULT_KM);
  mot_left_km = node->declare_parameter("mot_left_km", DEFAULT_KM);
  sub_motor = node->create_subscription<MotorVoltage>(
      MOTOR_TOPIC, MOTOR_QOS, [this](MotorVoltage::SharedPtr msg) { _last_motor = msg; });

  // Encoders
  _enc_msg.left = 0;
  _enc_msg.right = 0;
  pub_enc = node->create_publisher<EncoderState>(ENC_TOPIC, ENC_QOS);
  enc_loop = node->create_wall_timer(
      milliseconds(node->declare_parameter("enc_ms", EncoderProxy::DEFAULT_ENC_MS)),
      [this]() { pub_enc->publish(_enc_msg); });
}

void Motor::update(double dt) {
  _enc_msg.right += _last_motor->right * mot_righ_km * dt;
  _enc_msg.left += _last_motor->left * mot_left_km * dt;
}

void Motor::cleanup() { _clock.reset(); }

}  // namespace raubase::simu::plugins