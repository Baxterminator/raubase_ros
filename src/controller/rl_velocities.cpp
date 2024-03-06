#include <algorithm>
#include <cmath>

#include "controller/controller.hpp"

/**========================================================================
 * !                              Controller
 * This file contains the methods implementation for the controller node
 * right-left (RL) wheels computations (i.e. the conversion to the RL
 * velocities, the pid control and the saturation).
 *
 *========================================================================**/

namespace raubase::motor {

void Controller::computeVelocitiesRef() {
  state.vel_dif = wheel_base * state.turn_rate / 2;
  state.vright_ref = last_cmd->velocity + state.vel_dif;
  state.vleft_ref = last_cmd->velocity - state.vel_dif;
}

void Controller::computeRLVelocities() {
  // Compute new PID value
  voltage_cmd.right =
      vel_pid_right.pid(state.vright_ref, last_odometry->v_right, state.voltage_saturation);
  voltage_cmd.left =
      vel_pid_left.pid(state.vleft_ref, last_odometry->v_left, state.voltage_saturation);

  // Check for voltage saturation
  double vr = std::fabs(voltage_cmd.right), vl = std::fabs(voltage_cmd.left);
  state.voltage_saturation = (vr > max_voltage || vl > max_voltage);

  // If saturation, reduce non-saturating wheel accordingly
  if (state.voltage_saturation) {
    double k = max_voltage / ((vr > vl) ? vr : vl);
    voltage_cmd.right *= k;
    voltage_cmd.left *= k;
  }

  // Save for debug
  state.vright = voltage_cmd.right;
  state.vleft = voltage_cmd.left;
}

}  // namespace raubase::motor