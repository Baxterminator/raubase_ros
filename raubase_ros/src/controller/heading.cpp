#include <complex>
#include <rclcpp/logging.hpp>

#include "common/math/math.hpp"
#include "controller/controller.hpp"

namespace raubase::motor {

void VelocityController::computeHeadingRef(double dt) {
  switch (last_cmd->move_type) {
    case CmdMove::CMD_POS:
    case CmdMove::CMD_V_ANGLE:
      state.heading_ref = last_cmd->heading;
      break;
    case CmdMove::CMD_V_TR:
      state.heading_ref += last_cmd->turn_rate * dt;
      break;
    default:
      state.heading_ref = last_odometry->heading;
  }
}

void VelocityController::computeTurnRate(double dt) {
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "Got time %f", dt);
  // Compute new turn rate command
  state.turn_rate =
      head_pid->update(dt, state.heading_ref, last_odometry->heading, state.turnrate_saturation);

  // Saturation for the turn rate
  if (state.turnrate_saturation =
          (std::fabs(state.turn_rate) > state.max_turnrate || state.voltage_saturation);
      state.turnrate_saturation) {
    state.turn_rate = math::saturate(state.turn_rate, state.max_turnrate);
  }
}

}  // namespace raubase::motor