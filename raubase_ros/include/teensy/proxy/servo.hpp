#ifndef RAUBASE_SERVO_PROXY
#define RAUBASE_SERVO_PROXY

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

#include <rclcpp/subscription.hpp>

#include "raubase_msgs/msg/cmd_servo_position.hpp"
#include "raubase_msgs/msg/data_servo.hpp"
#include "teensy/interface/proxy_interface.hpp"

using raubase_msgs::msg::CmdServoPosition;
using raubase_msgs::msg::DataServo;

namespace raubase::teensy::proxy {

/**
 * @brief Proxy for receiving ROS servo commands et sending servo positions.
 */
class ServoProxy : public TeensyProxy {
  // =================================================================
  //                             Constants
  // =================================================================
 public:
  constchar NODE_NAME{"Servo Proxy"};     //< ROS Node name
  constchar TEENSY_COMP{"svo"};           //< Teensy component to subscribe from
  constchar TEENSY_MSG = TEENSY_COMP;     //< Teensy board receiving prefix
  constval int QOS{10};                   //< QOS for all components
  constchar SERVO_MSG{"servo %d %d %d"};  //< Command to send to the teensy

  struct Topics {
    constchar STATE_PUB{"sensor/servos"};
    constchar CMD_SUB{"control/set_servos"};
  };
  struct Params {
    constchar PROXY_ON{"servo_on"};      //< Is this proxy running ?
    constchar N_SERVOS{"servo_number"};  //< Number of servo presents
    constchar RATE_MS{"servo_rate_ms"};  //< Period between messages in ms
  };
  struct Default {
    constval bool PROXY_ON{false};  //< Is this proxy on ?
    constval int RATE_MS{40};       //< Default period between messages in ms
    constval int N_SERVOS{5};       //< Default number of servos
  };

  // =================================================================
  //                             Proxy Methods
  // =================================================================
 public:
  ServoProxy(SendingCallback _clbk) : TeensyProxy(_clbk, NODE_NAME) {}
  void setupParams(rclcpp::Node::SharedPtr) override;
  void setupSubscriptions() override;
  void decode(char*) override;

  static sptr<ServoProxy> make(SendingCallback _clbk) {
    return std::make_shared<ServoProxy>(_clbk);
  }

  // =================================================================
  //                          Component Methods
  // =================================================================
 protected:
  /**
   * @brief Receive the ROS command to set the servos at one position and transfer it to the Teensy
   * Board.
   */
  void setServoPosition(const CmdServoPosition::SharedPtr);

 public:
  // =================================================================
  //                             ROS Members
  // =================================================================
 protected:
  DataServo _servo_states;
  rclcpp::Publisher<DataServo>::SharedPtr state_pub;
  rclcpp::Subscription<CmdServoPosition>::SharedPtr cmd_sub;
  rclcpp::Clock::SharedPtr clock;  //< ROS clock for stamping the messages

  // =================================================================
  //                           Sensor Parameters
  // =================================================================
  bool _is_on;        //< Is this proxy on ?
  int _n_servos;      //< Number of servos
  int _refresh_rate;  //< The refresh rate for the distance sensors
};

}  // namespace raubase::teensy::proxy

#endif