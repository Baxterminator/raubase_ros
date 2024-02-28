#ifndef RAUBSE_MOTOR_PROXY
#define RAUBSE_MOTOR_PROXY

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

#include "robotbot_msgs/msg/motor_voltage.hpp"
#include "teensy/interface/proxy_interface.hpp"

using robotbot_msgs::msg::MotorVoltage;

namespace raubase::teensy::proxy {
/**
 * @brief Proxy for communicating with the motors.
 * It's essentially for commanding him.
 */
class MotorProxy : public TeensyProxy {
  // =================================================================
  //                             Constants
  // =================================================================
 public:
  static constexpr const char* NODE_NAME{"MotorProxy"};  //< ROS Node name

 protected:
  static constexpr const char* SUBSCRIBING_TOPIC{"set_voltage"};  //< Cmd subscribing topic
  static constexpr const char* MOTOR_CMD{"motv %f %f"};  // Cmd to send to control the motors
  static constexpr int QOS{10};                          //< QOS for all components

  // =================================================================
  //                             Proxy Methods
  // =================================================================
 public:
  MotorProxy(SendingCallback _clbk) : TeensyProxy(_clbk, NODE_NAME) {}
  void setupParams(rclcpp::Node::SharedPtr) override;
  void setupSubscriptions() override;
  void decode(char*) override{};

  void closeTeensy() override;

  // =================================================================
  //                          Component Methods
  // =================================================================
 private:
  void sendCmd(const MotorVoltage::SharedPtr, bool direct = false);

  // =================================================================
  //                             ROS Members
  // =================================================================
 protected:
  rclcpp::Subscription<MotorVoltage>::SharedPtr subscriber;  //< Heartbeat publisher
  rclcpp::Clock::SharedPtr clock;                            //< ROS clock for stamping the messages
  double _max_voltage;  //< Maximum voltage allowed by the motors
};
}  // namespace raubase::teensy::proxy

#endif