#include <complex>

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
      state.heading_ref = last_odometry->heading + last_cmd->turn_rate * dt;
      break;
    default:
      state.heading_ref = last_odometry->heading;
  }
}

void VelocityController::computeTurnRate(double dt) {
  // Compute new turn rate command
  state.turn_rate =
      head_pid->update(dt, state.heading_ref, last_odometry->heading, state.turnrate_saturation);

  // Saturation for the turn rate
  state.turnrate_saturation =
      (std::fabs(state.turn_rate) > max_turn_rate || state.voltage_saturation);
  if (state.turnrate_saturation) {
    state.turn_rate = math::saturate(state.turn_rate, max_turn_rate);
  }
}

}  // namespace raubase::motor