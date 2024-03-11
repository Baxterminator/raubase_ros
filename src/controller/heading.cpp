#include <complex>

#include "common/utils/math.hpp"
#include "controller/controller.hpp"

namespace raubase::motor {

void Controller::computeHeadingRef(double dt) {
  switch (last_cmd->move_type) {
    case MoveCmd::CMD_POS:
    case MoveCmd::CMD_V_ANGLE:
      state.heading_ref = last_cmd->heading;
      break;
    case MoveCmd::CMD_V_TR:
      state.heading_ref = last_odometry->heading + last_cmd->turn_rate * dt;
      break;
    default:
      state.heading_ref = last_odometry->heading;
  }
}

void Controller::computeTurnRate() {
  // Compute new turn rate command
  state.turn_rate =
      heading_pid.pid(state.heading_ref, last_odometry->heading, state.turnrate_saturation);

  // Saturation for the turn rate
  state.turnrate_saturation =
      (std::fabs(state.turn_rate) > max_turn_rate || state.voltage_saturation);
  if (state.turnrate_saturation) {
    state.turn_rate = math::saturate(state.turn_rate, max_turn_rate);
  }
}

}  // namespace raubase::motor