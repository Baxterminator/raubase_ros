#ifndef RAUBASE_CONTROLLER
#define RAUBASE_CONTROLLER

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
#include <memory>
#include <raubase_msgs/msg/controller_state.hpp>
#include <raubase_msgs/msg/motor_voltage.hpp>
#include <raubase_msgs/msg/move_cmd.hpp>
#include <raubase_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include "common/utils/upid.hpp"

using namespace rclcpp;
using namespace std::chrono;
using namespace std::chrono_literals;
using raubase_msgs::msg::ControllerState;
using raubase_msgs::msg::MotorVoltage;
using raubase_msgs::msg::MoveCmd;
using raubase_msgs::msg::Odometry;

namespace raubase::motor {

/**
 * @brief This class converts the position and velocity commands into voltage commands for the two
 * motors.
 *
 */
class Controller : public Node {
  // ==========================================================================
  //                                 Constants
  // ==========================================================================
 private:
  static constexpr const char* SUB_CMD_TOPIC{"/move"};
  static constexpr const char* SUB_ODOMETRY{"/odometry"};
  static constexpr const char* PUB_CMD_TOPIC{"/set_voltage"};
  static constexpr const char* PUB_STATE_TOPIC{"/state"};
  static constexpr const int QOS{10};

  // ----------------------------- Default values ---------------------------------
  // General parameters
  static constexpr int DEFAULT_FREQ = 100;            //< Frequency of the controller loop
  static constexpr float DEFAULT_MAX_V = 10.0;        //< Max voltage for commanding the motors
  static constexpr float DEFAULT_MAX_TR = 3.0;        //< Max turn rate for the robot (rad/s)
  static constexpr float DEFAULT_WHEEL_BASE = 0.243;  //< Distance between the wheels

  // Velocity PID
  static constexpr float DEFAULT_PID_V_KP = 7.0;   //< Velocity PID: prop. gain (V per (m/s))
  static constexpr float DEFAULT_PID_V_TD = 0.0;   //< Velocity PID: lead time constant (sec)
  static constexpr float DEFAULT_PID_V_AD = 1.0;   //< Velocity PID: lead alpha value
  static constexpr float DEFAULT_PID_V_TI = 0.05;  //< Velocity PID: integrator time constant (sec)

  // Heading PID
  static constexpr float DEFAULT_PID_H_KP = 10.0;  //< Heading PID: prop. gain (rad/s per rad)
  static constexpr float DEFAULT_PID_H_TD = 0.0;   //< Heading PID: lead time constant (sec)
  static constexpr float DEFAULT_PID_H_AD = 1.0;   //< Heading PID: lead alpha value
  static constexpr float DEFAULT_PID_H_TI = 0.;    //< Heading PID: integrator time constant (sec)

  // ==========================================================================
  //                                 Methods
  // ==========================================================================
 public:
  // ----------------------------- Life Cycle ---------------------------------
  Controller(NodeOptions);

  void terminate();

 private:
  /**
   * @brief Loop for launching the several PID computations and export the command.
   */
  void loop();

  // ----------------------------- Turn rate ----------------------------------
  /**
   * @brief Compute the heading reference;
   */
  void computeHeadingRef(double dt);

  /**
   * @brief Compute the turn rate from the last received command.
   */
  void computeTurnRate();

  // ----------------------------- Velocities ---------------------------------
  /**
   * @brief Compute the targeted RL velocities for the motors.
   */
  void computeVelocitiesRef();
  /**
   * @brief Compute the left-right wheels velocities command in Volts based on the reference.
   */
  void computeRLVelocities();

  // ==========================================================================
  //                                    Members
  // ==========================================================================
 private:
  // ------------------------ Parameters of the node --------------------------
  microseconds loop_period;  //< Duration between two loop runs
  double max_voltage;        //< Maximum voltage accepted by the motors
  double max_turn_rate;      //< Maximum turn rate allowed by the robot
  double wheel_base;         //< Distance between the two wheels
  bool debug;                //< Whether the node run on debug

  // ----------------------------- ROS Members --------------------------------
  TimerBase::SharedPtr fixed_loop;                  //< Controller loop
  Subscription<MoveCmd>::SharedPtr cmd_sub;         //< Command subscriber
  Subscription<Odometry>::SharedPtr odom_sub;       //< Odometry subscriber
  Publisher<MotorVoltage>::SharedPtr voltage_pub;   //< Motor voltage command publisher
  Publisher<ControllerState>::SharedPtr state_pub;  //< Internal state publisher

  // -------------------------------- Messages --------------------------------
  MoveCmd::SharedPtr last_cmd;        //< Last command received
  Odometry::SharedPtr last_odometry;  //< Last odometry received
  MotorVoltage voltage_cmd;           //< Command voltage message

  // ----------------------------------- PIDs ---------------------------------
  ControllerState state;  //< Internal state of the controller (as msg for debug publishing)
  PID vel_pid_right, vel_pid_left;  //< PIDs for controlling the motors velocity
  PID heading_pid;                  //< PID for controlling the heading
};

}  // namespace raubase::motor

#endif