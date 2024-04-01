#ifndef RAUBASE_EDGE_CONTROLLER
#define RAUBASE_EDGE_CONTROLLER

#include <chrono>
#include <raubase_msgs/msg/cmd_line_follower.hpp>
#include <raubase_msgs/msg/cmd_move.hpp>
#include <raubase_msgs/msg/data_gpio.hpp>
#include <raubase_msgs/msg/data_line_sensor.hpp>
#include <raubase_msgs/msg/result_edge.hpp>
#include <raubase_msgs/msg/set_controller_input.hpp>
#include <raubase_msgs/msg/set_line_sensor_config.hpp>
#include <raubase_msgs/msg/state_velocity_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <vector>

#include "common/math/controller.hpp"
#include "common/utils/types.hpp"

using namespace rclcpp;
using raubase_msgs::msg::CmdLineFollower;
using raubase_msgs::msg::CmdMove;
using raubase_msgs::msg::DataGPIO;
using raubase_msgs::msg::DataLineSensor;
using raubase_msgs::msg::ResultEdge;
using raubase_msgs::msg::SetControllerInput;
using raubase_msgs::msg::SetLineSensorConfig;
using raubase_msgs::msg::StateVelocityController;
using std::chrono::microseconds;

template <typename T>
inline std::vector<T> calib(ulong size, T value) {
  std::vector<T> out(size);
  for (ulong i = 0; i < size; i++) out[i] = value;
  return out;
}

namespace raubase::control {

/**
 * @brief This class read the line and try to follow it.
 */
class LineFollower : public Node {
  // ==========================================================================
  //                                 Constants
  // ==========================================================================
 private:
  constchar NODE_NAME{"egde_control"};
  constval int QOS{10};
  constval long MIN_DIFF_W_B{10};  //< The minimum difference between black and white calibration

 public:
  struct Params {
    constchar PID_FREQ{"pid_freq"};            //< Frequency of the controller loop
    constchar PID_KP{"pid_kp"};                //< PID: prop. gain (V per (m/s))
    constchar PID_TD{"pid_td"};                //< PID: lead time constant (sec)
    constchar PID_AD{"pid_ad"};                //< PID: lead alpha value
    constchar PID_TI{"pid_ti"};                //< PID: integrator time constant (sec)
    constchar MAX_TR{"max_turn_rate"};         //< Max turn rate for the robot (rad/s)
    constchar N_SENSORS{"n_sensors"};          //< Number of sensors for line detection
    constchar WIDTH{"width_m"};                //< Width of the full sensor board (in m)
    constchar WHITE_CALIB{"white_calib"};      //< White calibration values (in [0-1000])
    constchar BLACK_CALIB{"black_calib"};      //< Black calibration values (in [0-1000])
    constchar WHITE_THRES{"white_threshold"};  //< White threshold of normalized (in [0-1000])
  };
  struct Default {
    constval int FREQ{-1};          //< Frequency of the controller loop
    constval float PID_KP{40};      //< PID: prop. gain (V per (m/s))
    constval float PID_TD{0.3};     //< PID: lead time constant (sec)
    constval float PID_AD{1.5};     //< PID: lead alpha value
    constval float PID_TI{0};       //< PID: integrator time constant (sec)
    constval float MAX_TR{7};       //< Max turn rate for the robot (rad/s)
    constval int N_SENSORS{8};      //< Number of sensors for line detection
    constval float WIDTH{0.12};     //< Width of the sensor board (in m)
    constval int WHITE{1000};       //< Value for the default white calibration (in [0-1000])
    constval int BLACK{0};          //< Value for the default black calibration (in [0-1000])
    constval int W_THRESHOLD{250};  //< Threshold for detecting a white value (in [0-1000])
  };
  struct Topics {
    constchar SUB_CONTROLLER_STATE{"state/vcontroller"};
    constchar SUB_REF{"control/line_conf"};
    constchar SUB_LINE{"sensor/line"};
    constchar SUB_STOP{"stop"};  //< Stop signal
    constchar PUB_CMD{"control/move_edge"};
    constchar PUB_RESULT{"sensor/edge"};
    constchar PUB_DECLARE{"control/declare_input"};
    constchar PUB_LINE_SET{"control/line/set"};
  };

  // ==========================================================================
  //                                 Methods
  // ==========================================================================
 public:
  // ----------------------------- Life Cycle ---------------------------------
  LineFollower(NodeOptions);

  // -------------------------------- Setup -----------------------------------
 private:
  /**
   * @brief Set the up the controller to make the commands.
   */
  void setup_controller();

  /**
   * @brief Declare this controller as input for the mixer.
   */
  void declare_controller();

  /**
   * @brief Check the calibration values.
   */
  void check_calib();

  // ----------------------------- Edge computing -----------------------------
  /**
   * @brief Compute the edge
   */
  void compute_edge();

  /**
   * @brief Update the controller value based on the computed result.
   */
  void update_controller(double dt);

  /**
   * @brief Loop for launching the several PID computations and export the command.
   */
  void loop();

  // ==========================================================================
  //                                    Members
  // ==========================================================================
 private:
  // ------------------------ Parameters of the node --------------------------
  bool consuming = false;   //< If the controller is in consuming mode, it computes as soon as the
                            // data is received, else it updates at a precise frequency
  float max_turn_rate;      //< Maximum acceptable turn rate
  unsigned int n_sensors;   //< Number of sensors,
  float half_n_sensors;     //< Half of the number of sensors
  float btwn_m, center_m;   //< Space between and middle of the sensor board
  bool calib_valid = true;  //< Whether the calibration is good
  std::vector<long> white_calibration;  //< Calibration of the line sensor for white
  std::vector<long> black_calibration;  //< Calibration of the line sensor for black
  std::vector<float> factor;            //< Factor for converting raw value between 0 and 1000
  float white_threshold;                //< Threshold for detecting white value
  bool limited;                         //< If the controller is at saturation

  // ----------------------------- ROS Members --------------------------------
  TimerBase::SharedPtr fixed_loop;  //< Controller loop

  CmdMove move_cmd;
  Publisher<CmdMove>::SharedPtr move_pub;  //< Publisher for moving the robot

  Publisher<SetControllerInput>::SharedPtr input_declaration;

  Subscription<DataLineSensor>::SharedPtr sensor_sub;
  Subscription<DataGPIO>::SharedPtr stop_sub;  //< Stop signal subscriber
  DataLineSensor::SharedPtr last_data, last_data_used;
  bool last_data_has_been_used = false;

  Subscription<CmdLineFollower>::SharedPtr ref_sub;
  CmdLineFollower::SharedPtr last_cmd;

  Subscription<StateVelocityController>::SharedPtr controller_state;
  StateVelocityController::SharedPtr last_control_state;

  ResultEdge result;
  Publisher<ResultEdge>::SharedPtr result_pub;

  SetLineSensorConfig config;
  Publisher<SetLineSensorConfig>::SharedPtr sensor_config;

  // ----------------------------------- PIDs ---------------------------------
  math::ControlInterface::SharedPtr pid;  //< PID for controlling the heading
};

}  // namespace raubase::control

#endif