#ifndef RAUBASE_IMU_PROXY
#define RAUBASE_IMU_PROXY

/*
Copyright (C) 2017-2024 by DTU
Authors:
  Christan Andersen: jcan@dtu.dk
  Geoffrey Côte: geoffrey.cote@centraliens-nantes.org

The MIT License (MIT)  https://mit-license.org/

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the “Software”), to deal in the Software without
restriction, including without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies
or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <sensor_msgs/msg/imu.hpp>

#include "teensy/interface/proxy_interface.hpp"

using sensor_msgs::msg::Imu;

namespace raubase::teensy::proxy {
/**
 * @brief Proxy for converting IMU (MPU9250) messages into a ROS data message.
 */
class IMUProxy : public TeensyProxy {
  // =================================================================
  //                             Constants
  // =================================================================
 public:
  static constexpr const char* NODE_NAME{"IMUProxy"};  //< ROS Node name
  static constexpr const char* GYRO_COMP{"gyro0"};     //< Gyroscope component name (subscribe)
  static constexpr const char* GYRO_MSG = GYRO_COMP;   //< Gyroscope message (receive)
  static constexpr const char* ACC_COMP{"acc0"};       //< Accelerometer component name (subscribe)
  static constexpr const char* ACC_MSG = ACC_COMP;     //< Accelerometer message (receive)

 protected:
  static constexpr const char* PUBLISHING_TOPIC{"imu"};  //< Encoder state topic
  static constexpr int QOS{10};                          //< QOS for all components

  // =================================================================
  //                             Proxy Methods
  // =================================================================
 public:
  IMUProxy(SendingCallback _clbk) : TeensyProxy(_clbk, NODE_NAME) {}
  void setupParams(rclcpp::Node::SharedPtr) override;
  void setupSubscriptions() override;
  void decode(char*) override;

  void closeTeensy() override {}

  // =================================================================
  //                          Component Methods
  // =================================================================

 public:
  // =================================================================
  //                             ROS Members
  // =================================================================
 protected:
  Imu _msg;                                     //< Message to send
  rclcpp::Publisher<Imu>::SharedPtr publisher;  //< Heartbeat publisher
  rclcpp::Clock::SharedPtr clock;               //< ROS clock for stamping the messages
  bool _gyro_updated = false;                   //< Flag if gyro is updated (to publish the message)
  bool _acc_updated = false;  //< Flag if accelerometer is updated (to publish message)

  // =================================================================
  //                           Sensor Parameters
  // =================================================================
  int refresh_rate;  //< The refresh rate for the encoders
};

}  // namespace raubase::teensy::proxy

#endif