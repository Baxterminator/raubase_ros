#include "teensy/proxy/imu.hpp"

namespace raubase::teensy::proxy {

void IMUProxy::setupParams(rclcpp::Node::SharedPtr node) {
  RCLCPP_INFO(logger, "Initializing proxy %s", NODE_NAME);

  // Declaring parameters for the encoder
  _on = node->declare_parameter("imu_on", true);
  refresh_rate = node->declare_parameter("imu_ms", 25);

  // Initializing working components
  publisher = node->create_publisher<Imu>(PUBLISHING_TOPIC, QOS);
  clock = node->get_clock();
}

void IMUProxy::setupSubscriptions() {
  if (!_on) return;
  RCLCPP_INFO(logger, "Initializing proxy %s", NODE_NAME);
  subscribeTeensyComponent(GYRO_COMP, refresh_rate);
  subscribeTeensyComponent(ACC_COMP, refresh_rate);
}

void IMUProxy::decode(char* msg) {
  // like: gyro0 <x> <y> <z>
  //   or: acc0 <x> <y> <z>
  if (strlen(msg) < 5) return;
  const char* p1 = msg + 5;

  if (std::strncmp(msg, GYRO_MSG, std::strlen(GYRO_MSG))) {
    _msg.orientation.x = strtod(p1, (char**)&p1);
    _msg.orientation.y = strtod(p1, (char**)&p1);
    _msg.orientation.z = strtod(p1, (char**)&p1);
    _msg.orientation.w = 0;

    _gyro_updated = true;
  } else if (std::strncmp(msg, ACC_MSG, std::strlen(ACC_MSG))) {
    _msg.linear_acceleration.x = strtod(p1, (char**)&p1);
    _msg.linear_acceleration.y = strtod(p1, (char**)&p1);
    _msg.linear_acceleration.z = strtod(p1, (char**)&p1);

    _acc_updated = true;
  }

  // If whole IMU data is received, send the message
  if (_gyro_updated && _acc_updated) {
    _msg.header.stamp = clock->now();
    _gyro_updated = false;
    _acc_updated = false;

    publisher->publish(_msg);
  }
}

}  // namespace raubase::teensy::proxy