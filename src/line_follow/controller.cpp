#include "controller/edge_controller.hpp"

namespace raubase::control {

LineFollower::LineFollower(NodeOptions opts) : Node(NODE_NAME, opts) {
  loop_period =
      microseconds((long)std::fabs(1. / declare_parameter("pid_freq", DEFAULT_FREQ) * 1E6));
  pid.setup(loop_period.count() * 1E-6, declare_parameter("pid_kp", DEFAULT_PID_KP),
            declare_parameter("pid_td", DEFAULT_PID_TD),
            declare_parameter("pid_ad", DEFAULT_PID_AD),
            declare_parameter("pid_ti", DEFAULT_PID_TI));
  max_turn_rate = declare_parameter("max_turn_rate", DEFAULT_MAX_TR);
  n_sensors = declare_parameter("n_sensors", DEFAULT_N_SENSORS);
  space_btwn = declare_parameter("space_btwn_m", DEFAULT_SENSOR_SPACED);

  RCLCPP_INFO(get_logger(), "Launching line follower unit with a period of %zuµs",
              loop_period.count());

  move_pub = create_publisher<CmdMove>(PUB_CMD_TOPIC, QOS);
  fixed_loop = create_wall_timer(loop_period, std::bind(&LineFollower::loop, this));
}

}  // namespace raubase::control

int main(int argc, char** argv) { return 0; }