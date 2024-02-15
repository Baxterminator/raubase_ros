#ifndef RAUBASE_ENCODER_PROXY
#define RAUBASE_ENCODER_PROXY

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

#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include "robotbot_msgs/msg/encoder_state.hpp"
#include "teensy/interface/proxy_interface.hpp"

using robotbot_msgs::msg::EncoderState;

namespace raubase::teensy::proxy {

/**
 * @brief Proxy for converting Teensy Encoder messages into a ROS data message.
 */
class EncoderProxy : public TeensyProxy {
  // =================================================================
  //                             Constants
  // =================================================================
 public:
  static constexpr const char* NODE_NAME{"EncoderProxy"};  //< ROS Node name
  static constexpr const char* TEENSY_COMP{"enc"};
  static constexpr const char* TEENSY_MSG = TEENSY_COMP;  //< Teensy board component name

 protected:
  static constexpr const char* PUBLISHING_TOPIC{"encoders"};  //< Encoder state topic
  static constexpr int QOS{10};                               //< QOS for all components

  // =================================================================
  //                             Proxy Methods
  // =================================================================
 public:
  EncoderProxy(SendingCallback _clbk) : TeensyProxy(_clbk, NODE_NAME) {}
  void setup(rclcpp::Node::SharedPtr) override;
  void decode(char*) override;

  // =================================================================
  //                          Component Methods
  // =================================================================
  void configEncoderReversing();

 public:
  // =================================================================
  //                             ROS Members
  // =================================================================
 protected:
  rclcpp::Publisher<EncoderState>::SharedPtr publisher;

  // =================================================================
  //                           Sensor Parameters
  // =================================================================
  int refresh_rate;  //< The refresh rate for the encoders
  bool reverse_enc;  //< Whether the encoder are reverse or not
};

}  // namespace raubase::teensy::proxy

#endif