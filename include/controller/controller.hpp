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
#include <raubase_msgs/msg/cmd_motor_voltage.hpp>
#include <raubase_msgs/msg/cmd_move.hpp>
#include <raubase_msgs/msg/result_odometry.hpp>
#include <raubase_msgs/msg/state_velocity_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include "common/math/controller.hpp"
#include "common/utils/types.hpp"

using namespace rclcpp;
using namespace std::chrono;
using namespace std::chrono_literals;
using raubase_msgs::msg::CmdMotorVoltage;
using raubase_msgs::msg::CmdMove;
using raubase_msgs::msg::ResultOdometry;
using raubase_msgs::msg::StateVelocityController;

namespace raubase::motor {

/**
 * @brief This class converts the position and velocity commands into voltage commands for the two
 * motors.
 *
 */
class VelocityController : public Node {
  // ==========================================================================
  //                                 Constants
  // ==========================================================================
 public:
  constchar NODE_NAME{"controller"};   //< Name of the ROS node
  static constexpr const int QOS{10};  //< Default QoS

  struct Topics {
    constchar SUB_CMD{"control/move"};         //< Where to grab commands
    constchar SUB_ODOMETRY{"state/odometry"};  //< Where to grab odometry
    constchar PUB_CMD{"control/set_voltage"};  //< Where to publish voltage commands
    constchar PUB_STATE{"state/vcontroller"};  //< Where to publish controller state
  };
  struct Params {
    constchar PID_FREQ{"pid_freq"};         //< PID frequency
    constchar MAX_VOLT{"max_volt"};         //< Maximum voltage for the motors (in V)
    constchar MAX_TR{"max_turn_rate"};      //< Maximum turn rate for the robot (in rad/s)
    constchar WHEEL_BASE{"wheel_base_m"};   //< Distance between the wheels (in m)
    constchar DEBUG{"debug"};               //< If in debug mode (share state)
    constchar MAX_ACC{"max_acceleration"};  //< Maximum wheel acceleration (m/s²)

    // PIDS
    constchar PID_V_KP{"vpid_kp"};  //< Velocity PID: prop. gain (V per (m/s))
    constchar PID_V_TD{"vpid_td"};  //< Velocity PID: lead time constant (sec)
    constchar PID_V_AD{"vpid_ad"};  //< Velocity PID: lead alpha value
    constchar PID_V_TI{"vpid_ti"};  //< Velocity PID: integrator time constant (sec)
    constchar PID_H_KP{"hpid_kp"};  //< Heading PID: prop. gain (rad/s per rad)
    constchar PID_H_TD{"hpid_td"};  //< Heading PID: lead time constant (sec)
    constchar PID_H_AD{"hpid_ad"};  //< Heading PID: lead alpha value
    constchar PID_H_TI{"hpid_ti"};  //< Heading PID: integrator time constant (sec)
  };
  struct Default {
    constval int FREQ = -1;             //< Frequency of the controller loop
    constval float MAX_VOLT = 10.0;     //< Max voltage for commanding the motors
    constval float MAX_TR = 3.0;        //< Max turn rate for the robot (rad/s)
    constval float WHEEL_BASE = 0.243;  //< Distance between the wheels (in m)
    constval bool IN_DEBUG = true;      //< Whether we are in debug mode
    constval float MAX_ACC{1.0};        //< Maximum wheel acceleration (m/s²), -1 for disabling

    // PIDS
    constval float PID_V_KP = 7.0;   //< Velocity PID: prop. gain (V per (m/s))
    constval float PID_V_TD = 0.0;   //< Velocity PID: lead time constant (sec)
    constval float PID_V_AD = 1.0;   //< Velocity PID: lead alpha value
    constval float PID_V_TI = 0.05;  //< Velocity PID: integrator time constant (sec)
    constval float PID_H_KP = 10.0;  //< Heading PID: prop. gain (rad/s per rad)
    constval float PID_H_TD = 0.0;   //< Heading PID: lead time constant (sec)
    constval float PID_H_AD = 1.0;   //< Heading PID: lead alpha value
    constval float PID_H_TI = 0.;    //< Heading PID: integrator time constant (sec)
  };

  // ==========================================================================
  //                                 Methods
  // ==========================================================================
 public:
  // ----------------------------- Life Cycle ---------------------------------
  VelocityController(NodeOptions);

  void terminate();

 private:
  /**
   * @brief Initialize the internal controllers
   *
   */
  void init_controllers();

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
  void computeTurnRate(double dt);

  // ----------------------------- Velocities ---------------------------------
  /**
   * @brief Compute the targeted RL velocities for the motors.
   */
  void computeVelocitiesRef(double dt);
  /**
   * @brief Compute the left-right wheels velocities command in Volts based on the reference.
   */
  void computeRLVelocities(double dt);

  // ==========================================================================
  //                                    Members
  // ==========================================================================
 private:
  // ------------------------ Parameters of the node --------------------------
  bool consuming = false;  //< If the controller is in consuming mode, it computes as soon as the
                           // data is received, else it updates at a precise frequency
  double wheel_base;       //< Distance between the two wheels
  bool debug;              //< Whether the node run on debug
  float max_acc;           //< Maximum wheel acceleration (m/s²)

  // ----------------------------- ROS Members --------------------------------
  TimerBase::SharedPtr fixed_loop;                          //< Controller loop
  Subscription<CmdMove>::SharedPtr cmd_sub;                 //< Command subscriber
  Subscription<ResultOdometry>::SharedPtr odom_sub;         //< ResultOdometry subscriber
  Publisher<CmdMotorVoltage>::SharedPtr voltage_pub;        //< Motor voltage command publisher
  Publisher<StateVelocityController>::SharedPtr state_pub;  //< Internal state publisher

  // -------------------------------- Messages --------------------------------
  CmdMove::SharedPtr last_cmd;              //< Last command received
  ResultOdometry::SharedPtr last_odometry;  //< Last odometry received
  CmdMotorVoltage voltage_cmd;              //< Command voltage message
  StateVelocityController state;  //< Internal state of the controller (as msg for debug publishing)

  // ----------------------------------- PIDs ---------------------------------
  math::ControlInterface::SharedPtr vel_pid_right,
      vel_pid_left;                            //< PIDs for controlling the motors velocity
  math::ControlInterface::SharedPtr head_pid;  //< PID for controlling the heading
};

}  // namespace raubase::motor

#endif