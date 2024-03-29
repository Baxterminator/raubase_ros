#include "controller/controller.hpp"

#include <chrono>
#include <cmath>
#include <rclcpp/logging.hpp>

#include "common/math/fixed_pid_control.hpp"
#include "common/math/pid_control.hpp"
#include "common/utils/types.hpp"
#include "common/utils/utime.hpp"

/**========================================================================
 * !                              Controller
 * This file contains the methods implementation for the controller node
 * lifecycle (init, main loop, destructor, ...). See the others files for
 * other part of the computations for the controller.
 *
 *========================================================================**/

namespace raubase::motor {

VelocityController::VelocityController(NodeOptions opts) : Node(NODE_NAME, opts) {
  // General initialization
  state.vleft_ref = 0;
  state.vright_ref = 0;
  state.heading_ref = 0;
  state.turn_rate = 0;
  state.vel_dif = 0;
  state.voltage_saturation = false;
  state.turnrate_saturation = false;

  last_odometry = std::make_shared<ResultOdometry>();
  last_odometry->v_left = 0;
  last_odometry->v_right = 0;
  last_odometry->heading = 0;

  last_cmd = std::make_shared<CmdMove>();
  last_cmd->move_type = CmdMove::CMD_V_TR;
  last_cmd->turn_rate = 0;
  last_cmd->velocity = 0;

  // Initiate parameters
  state.max_voltage = declare_parameter(Params::MAX_VOLT, Default::MAX_VOLT);
  state.max_turnrate = declare_parameter(Params::MAX_TR, Default::MAX_TR);
  state.max_acceleration = declare_parameter(Params::MAX_ACC, Default::MAX_ACC);

  wheel_base = declare_parameter(Params::WHEEL_BASE, Default::WHEEL_BASE);
  debug = declare_parameter(Params::DEBUG, Default::IN_DEBUG);

  // Initiate Controllers
  init_controllers();

  // Initiate ROS components
  cmd_sub = create_subscription<CmdMove>(Topics::SUB_CMD, QOS,
                                         [this](const CmdMove::SharedPtr cmd) { last_cmd = cmd; });
  odom_sub = create_subscription<ResultOdometry>(Topics::SUB_ODOMETRY, QOS,
                                                 [this](const ResultOdometry::SharedPtr odm) {
                                                   last_odometry = odm;
                                                   if (consuming) loop();
                                                 });
  voltage_pub = create_publisher<CmdMotorVoltage>(Topics::PUB_CMD, QOS);
  if (debug) state_pub = create_publisher<StateVelocityController>(Topics::PUB_STATE, QOS);
}

void VelocityController::init_controllers() {
  int freq = declare_parameter(Params::PID_FREQ, Default::FREQ);

  // Velocity controller constants
  float vel_kp = declare_parameter(Params::PID_V_KP, Default::PID_V_KP);
  float vel_td = declare_parameter(Params::PID_V_TD, Default::PID_V_TD);
  float vel_ad = declare_parameter(Params::PID_V_AD, Default::PID_V_AD);
  float vel_ti = declare_parameter(Params::PID_V_TI, Default::PID_V_TI);

  // Heading controller constants
  float heading_kp = declare_parameter(Params::PID_H_KP, Default::PID_H_KP);
  float heading_td = declare_parameter(Params::PID_H_TD, Default::PID_H_TD);
  float heading_ad = declare_parameter(Params::PID_H_AD, Default::PID_H_AD);
  float heading_ti = declare_parameter(Params::PID_H_TI, Default::PID_H_TI);

  // If controller works on "as soon as we have the odometry"
  if (freq == -1) {
    consuming = true;
    RCLCPP_INFO(get_logger(), "Launching velocity controller unit in consuming mode");
    vel_pid_right = math::PILeadController::make(vel_kp, vel_td, vel_ad, vel_ti);
    vel_pid_left = math::PILeadController::make(vel_kp, vel_td, vel_ad, vel_ti);
    head_pid = math::PILeadController::make(heading_kp, heading_td, heading_ad, heading_ti);
  }
  // Else the controller is at a fixed speed
  else {
    double period = 1.0 / ((double)freq);
    RCLCPP_INFO(get_logger(), "Launching velocity controller unit with a period of %zuµs",
                (long)std::floor(period * 1E6));
    vel_pid_right = math::FixedPILeadController::make(vel_kp, vel_td, vel_ad, vel_ti, period);
    vel_pid_left = math::FixedPILeadController::make(vel_kp, vel_td, vel_ad, vel_ti, period);
    head_pid =
        math::FixedPILeadController::make(heading_kp, heading_td, heading_ad, heading_ti, period);
    fixed_loop = create_wall_timer(microseconds((long)std::floor(period * 1E6)),
                                   std::bind(&VelocityController::loop, this));
  }
  head_pid->set_fold_angle(true);
}

void VelocityController::terminate() {
  voltage_cmd.right = 0;
  voltage_cmd.left = 0;
  voltage_pub->publish(voltage_cmd);
}

///////////////////////////////////////////////////////////////////////////////

void VelocityController::loop() {
  static UTime loop_time("now");

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), THROTTLE_DUR, "Loop");
  float dt = loop_time.getTimePassed();

  // Compute the Turn Rate to apply
  computeHeadingRef(dt);
  computeTurnRate(dt);

  // Compute the RL velocities
  computeVelocitiesRef(dt);
  computeRLVelocities(dt);

  // Publish the message
  state.stamp = get_clock()->now();
  voltage_pub->publish(voltage_cmd);
  if (debug) state_pub->publish(state);
  loop_time.now();
}

}  // namespace raubase::motor

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<raubase::motor::VelocityController>(NodeOptions{});

  while (rclcpp::ok()) rclcpp::spin_some(controller);
  controller->terminate();
  rclcpp::spin_some(controller);

  rclcpp::shutdown();
  return 0;
}