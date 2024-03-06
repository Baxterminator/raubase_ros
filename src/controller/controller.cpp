#include "controller/controller.hpp"

#include <rclcpp/logging.hpp>

#include "common/utime.hpp"

/**========================================================================
 * !                              Controller
 * This file contains the methods implementation for the controller node
 * lifecycle (init, main loop, destructor, ...). See the others files for
 * other part of the computations for the controller.
 *
 *========================================================================**/

namespace raubase::motor {

Controller::Controller(NodeOptions opts) : Node("controller", opts) {
  // General initialization
  state.vleft_ref = 0;
  state.vright_ref = 0;
  state.heading_ref = 0;
  state.turn_rate = 0;
  state.vel_dif = 0;
  state.voltage_saturation = false;
  state.turnrate_saturation = false;

  last_odometry = std::make_shared<Odometry>();
  last_odometry->v_left = 0;
  last_odometry->v_right = 0;
  last_odometry->heading = 0;

  last_cmd = std::make_shared<MoveCmd>();
  last_cmd->move_type = MoveCmd::CMD_V_TR;
  last_cmd->turn_rate = 0;
  last_cmd->velocity = 0;

  // Initiate parameters
  loop_period =
      microseconds((long)std::fabs(1. / declare_parameter("pid_freq", DEFAULT_FREQ) * 1E6));
  max_voltage = declare_parameter("max_volt", DEFAULT_MAX_V);
  state.max_voltage = max_voltage;
  max_turn_rate = declare_parameter("max_turn_rate", DEFAULT_MAX_TR);
  state.max_turnrate = max_turn_rate;
  wheel_base = declare_parameter("wheel_base", DEFAULT_WHEEL_BASE);
  debug = declare_parameter("debug", true);

  // Initiate PIDs
  vel_pid_right.setup(loop_period.count() * 1E-6, declare_parameter("vpid_kp", DEFAULT_PID_V_KP),
                      declare_parameter("vpid_td", DEFAULT_PID_V_TD),
                      declare_parameter("vpid_ad", DEFAULT_PID_V_AD),
                      declare_parameter("vpid_ti", DEFAULT_PID_V_TI));
  vel_pid_left.setup(loop_period.count() * 1E-6, get_parameter("vpid_kp").as_double(),
                     get_parameter("vpid_td").as_double(), get_parameter("vpid_ad").as_double(),
                     get_parameter("vpid_ti").as_double());
  heading_pid.setup(loop_period.count() * 1E-6, declare_parameter("hpid_kp", DEFAULT_PID_H_KP),
                    declare_parameter("hpid_td", DEFAULT_PID_H_TD),
                    declare_parameter("hpid_ad", DEFAULT_PID_H_AD),
                    declare_parameter("hpid_ti", DEFAULT_PID_H_TI));
  heading_pid.doAngleFolding(true);

  // Initiate ROS components
  RCLCPP_INFO(get_logger(), "Launching controller unit with a period of %zuµs",
              loop_period.count());
  cmd_sub = create_subscription<MoveCmd>(SUB_CMD_TOPIC, QOS,
                                         [this](const MoveCmd::SharedPtr cmd) { last_cmd = cmd; });
  odom_sub = create_subscription<Odometry>(
      SUB_ODOMETRY, QOS, [this](const Odometry::SharedPtr odm) { last_odometry = odm; });
  voltage_pub = create_publisher<MotorVoltage>(PUB_CMD_TOPIC, QOS);
  if (debug) state_pub = create_publisher<ControllerState>(PUB_STATE_TOPIC, QOS);
  fixed_loop = create_wall_timer(loop_period, std::bind(&Controller::loop, this));
}

void Controller::terminate() {
  voltage_cmd.right = 0;
  voltage_cmd.left = 0;
  voltage_pub->publish(voltage_cmd);
}

void Controller::loop() {
  static UTime loop_time("now");

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Loop");

  // Compute the Turn Rate to apply
  computeHeadingRef(loop_time.getTimePassed());
  computeTurnRate();

  // Compute the RL velocities
  computeVelocitiesRef();
  computeRLVelocities();

  // Publish the message
  voltage_pub->publish(voltage_cmd);
  if (debug) state_pub->publish(state);
  loop_time.now();
}

}  // namespace raubase::motor

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<raubase::motor::Controller>(NodeOptions{});

  while (rclcpp::ok()) rclcpp::spin_some(controller);
  controller->terminate();
  rclcpp::spin_some(controller);

  rclcpp::shutdown();
  return 0;
}