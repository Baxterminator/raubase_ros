#include "simulator/plugins/Motor.hpp"

namespace raubase::simu::plugins {

void Motor::setup(rclcpp::Node::SharedPtr node) {
  // ROS
  _clock = node->get_clock();

  // Motor
  _last_motor = std::make_shared<CmdMotorVoltage>();
  _last_motor->right = 0;
  _last_motor->left = 0;
  mot_righ_km = node->declare_parameter("mot_right_km", DEFAULT_KM);
  mot_left_km = node->declare_parameter("mot_left_km", DEFAULT_KM);
  sub_motor = node->create_subscription<CmdMotorVoltage>(
      MOTOR_TOPIC, MOTOR_QOS, [node, this](CmdMotorVoltage::SharedPtr msg) {
        _last_motor = msg;
        RCLCPP_INFO(node->get_logger(), "New motor command (R %f, L %f)!", _last_motor->right,
                    _last_motor->left);
      });

  // Encoders
  _enc_msg.left = 0;
  _enc_msg.right = 0;
  pub_enc = node->create_publisher<DataEncoder>(ENC_TOPIC, ENC_QOS);
  enc_loop = node->create_wall_timer(
      milliseconds(node->declare_parameter("enc_ms", EncoderProxy::DEFAULT_ENC_MS)),
      [this]() { pub_enc->publish(_enc_msg); });
}

void Motor::update(double dt) {
  right_wheel += _last_motor->right * mot_righ_km * dt;
  left_wheel += _last_motor->left * mot_left_km * dt;

  _enc_msg.stamp = _clock->now();
  _enc_msg.right = robot->RW_pos2enc(right_wheel);
  _enc_msg.left = robot->LW_pos2enc(left_wheel);
}

void Motor::cleanup() { _clock.reset(); }

}  // namespace raubase::simu::plugins