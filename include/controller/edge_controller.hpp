#ifndef RAUBASE_EDGE_CONTROLLER
#define RAUBASE_EDGE_CONTROLLER

#include <chrono>
#include <raubase_msgs/msg/cmd_move.hpp>
#include <raubase_msgs/msg/data_line_sensor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include "common/utils/upid.hpp"

using namespace rclcpp;
using raubase_msgs::msg::CmdMove;
using raubase_msgs::msg::DataLineSensor;
using std::chrono::microseconds;

namespace raubase::control {

/**
 * @brief This class converts the position and velocity commands into voltage commands for the two
 * motors.
 */
class LineFollower : public Node {
  // ==========================================================================
  //                                 Constants
  // ==========================================================================
 private:
  static constexpr const int QOS{10};

 public:
  // ---------------------------------- ROS -----------------------------------
  static constexpr const char* NODE_NAME{"egde_control"};
  static constexpr const char* PUB_CMD_TOPIC{"/move"};

  // ----------------------------- Default values -----------------------------
  // General parameters
  static constexpr int DEFAULT_FREQ = 500;              //< Frequency of the controller loop
  static constexpr float DEFAULT_MAX_TR = 7.0;          //< Max turn rate for the robot (rad/s)
  static constexpr int DEFAULT_N_SENSORS = 8;           //< Number of sensors for line detection
  static constexpr float DEFAULT_SENSOR_SPACED = 0.12;  //< Space between sensor(in m)

  // PID
  static constexpr float DEFAULT_PID_KP = 40.0;  //< PID: prop. gain (V per (m/s))
  static constexpr float DEFAULT_PID_TD = 0.3;   //< PID: lead time constant (sec)
  static constexpr float DEFAULT_PID_AD = 1.5;   //< PID: lead alpha value
  static constexpr float DEFAULT_PID_TI = 0.0;   //< PID: integrator time constant (sec)
  // ==========================================================================
  //                                 Methods
  // ==========================================================================
 public:
  // ----------------------------- Life Cycle ---------------------------------
  LineFollower(NodeOptions);

 private:
  /**
   * @brief Loop for launching the several PID computations and export the command.
   */
  void loop();

  // ==========================================================================
  //                                    Members
  // ==========================================================================
 private:
  // ------------------------ Parameters of the node --------------------------
  microseconds loop_period;  //< Duration between two loop runs
  float max_turn_rate;       //< Maximum acceptable turn rate
  unsigned int n_sensors;    //< Number of ensors
  float space_btwn;          //< Space between sensors
  bool debug;                //< Whether the node run on debug

  // ----------------------------- ROS Members --------------------------------
  TimerBase::SharedPtr fixed_loop;  //< Controller loop
  Subscription<DataLineSensor>::SharedPtr sensor_sub;
  Publisher<CmdMove>::SharedPtr move_pub;  //< Publisher for moving the robot

  // ----------------------------------- PIDs ---------------------------------
  PID pid;  //< PID for controlling the heading
};

}  // namespace raubase::control

#endif