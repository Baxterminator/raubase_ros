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
#include "common/utils/types.hpp"

using namespace rclcpp;
using namespace raubase_msgs;
using raubase::kinematics::TwoWheeledRoverKinematics;
using std::chrono::microseconds;

namespace raubase::loc {
class Odometry : public Node {
  // ==========================================================================
  //                                 Constants
  // ==========================================================================
 public:
  constchar NODE_NAME{"odometry"};     //< Name of the node
  constval int QOS{10};                //< QoS for all participants
  constval int MAX_TICK_CHANGE{1000};  //< Max acceptable tick change

  struct Topics {
    constchar SUB_ENC{"sensor/encoders"};  //< Topics of the encoders state
    constchar PUB_ODOM{"state/odometry"};  //< Topic for sharing the odometry
  };
  struct Params {
    constchar ODOM_FREQ{"odom_freq"};  //< Default odometry frequency

    constchar R_WHEEL_D{"right_wheel_diameter_m"};  //< Right wheel diameter (in m)
    constchar R_WHEEL_RATIO{"right_gear_ratio"};    //< Right wheel gear ratio
    constchar R_WHEEL_TPR{"right_tick_per_rev"};    //< Right wheel number of tick per revolution
    constchar L_WHEEl_D{"left_wheel_diameter_m"};   //< Left wheel diameter (in m)
    constchar L_WHEEL_RATIO{"left_gear_ratio"};     //< Left wheel gear ratio
    constchar L_WHEEL_TPR{"left_tick_per_rev"};     //< Left wheel number of tick per revolution
    constchar BASE_WIDTH{"base_width_m"};           //< Width between the two wheels (in m)
    constchar F_LON{"flon"};                        //< Longitudinal error coefficient
    constchar F_LAT{"flat"};                        //< Lateral error coefficient
  };

  // Default values
  struct Default {
    constval int ODOM_FREQ{-1};  //< Default odometry frequency

    constval float WHEEL_D{0.146};    //< Default wheel diameter (in meters)
    constval float GEAR_RATIO{19.0};  //< Default gear ratio
    constval float TICK_PER_REV{68};  //< Default number of ticks per encoder revolution
    constval double DEF_BASE{0.243};  //< Default wheel base width (in meters)
    constval float F_LAT{1.0};        //< Lateral error coefficient
    constval float F_LON{1.0};        //< Longitudinal error coefficient
  };

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