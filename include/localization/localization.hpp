#ifndef RAUBASE_LOCALIZATION
#define RAUBASE_LOCALIZATION

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

#include <chrono>
#include <raubase_msgs/msg/data_encoder.hpp>
#include <raubase_msgs/msg/result_odometry.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include "common/robot/kinematics.hpp"

using namespace rclcpp;
using namespace raubase_msgs;
using raubase::kinematics::TwoWheeledRoverKinematics;
using raubase::kinematics::Wheel;
using std::chrono::microseconds;

namespace raubase::loc {
class Odometry : public Node {
  // ==========================================================================
  //                                 Constants
  // ==========================================================================
  static constexpr const char* NODE_NAME{"odometry"};              //< Name of the node
  static constexpr const char* SUB_ENC_TOPIC{"/teensy/encoders"};  //< Topics of the encoders state
  static constexpr const char* PUB_ODOM_TOPIC{"odometry"};  //< Topic for sharing the odometry
  static constexpr int QOS{10};                             //< QoS for all participants
  static constexpr int MAX_TICK_CHANGE{1000};               //< Max acceptable tick change

  // Default values
  static constexpr int DEF_ODOM_FREQ{100};       //< Default odometry frequency
  static constexpr double DEF_GEAR_RATIO{19.0};  //< Default gear ratio
  static constexpr double DEF_WHEEL_D{0.146};    //< Default wheel diameter (in meters)
  static constexpr int DEF_TICK_PER_REV{68};     //< Default number of ticks per encoder revolution
  static constexpr double DEF_BASE{0.243};       //< Default wheel base width (in meters)

  // ==========================================================================
  //                                 Methods
  // ==========================================================================
 public:
  // ----------------------------- Life Cycle ---------------------------------
  Odometry(NodeOptions);

 private:
  // ----------------------------- Callbacks ---------------------------------
  /**
   * @brief Update the odometry based on the latest message.
   */
  void updateOdometry();

  // ==========================================================================
  //                                    Members
  // ==========================================================================
 private:
  // ------------------------------- Parameters -------------------------------
  microseconds odom_loop_period;  //< Period for the loop duration
  double gear;                    //< Gear ratio for the reducter (input:output = gear:1)
  double wheel_d;                 //< Wheel diameter (in meters)
  int tick_per_rev;               //< Number of tick per revolution for the encoder
  double base;                    //< Base width (distance between the wheels, in meters)
  double dist_per_tick;           //< Distance between two wheel ticks
  TwoWheeledRoverKinematics::SharedPtr robot;

  // ---------------------------------- ROS -----------------------------------
  msg::DataEncoder::SharedPtr last_enc, last_enc_used;
  bool last_enc_has_been_used = false;
  msg::ResultOdometry odom_msg;
  Subscription<msg::DataEncoder>::SharedPtr encoder_sub;
  Publisher<msg::ResultOdometry>::SharedPtr odom_pub;
  TimerBase::SharedPtr odom_loop;
};

}  // namespace raubase::loc

#endif