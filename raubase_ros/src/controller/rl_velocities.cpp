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

void VelocityController::computeVelocitiesRef(double dt) {
  state.vel_dif = wheel_base * state.turn_rate / 2;

  float v_right = last_cmd->velocity + state.vel_dif;
  float v_left = last_cmd->velocity - state.vel_dif;

  // Compute accelerations
  state.aright_ref = (v_right - state.vright_ref) / dt;
  state.aleft_ref = (v_left - state.vleft_ref) / dt;
  state.acc_coeff = std::max({std::fabs(state.aleft_ref) / state.max_acceleration,
                              std::fabs(state.aright_ref) / state.max_acceleration, 1.});

  // Compute new velocity
  state.vright_ref += (v_right - state.vright_ref) / state.acc_coeff;
  state.vleft_ref += (v_left - state.vleft_ref) / state.acc_coeff;
  state.acc_saturation = state.acc_coeff > 1.;
}

void VelocityController::computeRLVelocities(double dt) {
  // Compute new PID value
  voltage_cmd.right =
      vel_pid_right->update(dt, state.vright_ref, last_odometry->v_right, state.voltage_saturation);
  voltage_cmd.left =
      vel_pid_left->update(dt, state.vleft_ref, last_odometry->v_left, state.voltage_saturation);

  // Check for voltage saturation
  double vr = std::fabs(voltage_cmd.right), vl = std::fabs(voltage_cmd.left);
  state.voltage_saturation = (vr > state.max_voltage || vl > state.max_voltage);

  // If saturation, reduce non-saturating wheel accordingly
  if (state.voltage_saturation) {
    double k = state.max_voltage / ((vr > vl) ? vr : vl);
    voltage_cmd.right *= k;
    voltage_cmd.left *= k;
  }

  // Save for debug
  state.volt_right = voltage_cmd.right;
  state.volt_left = voltage_cmd.left;
}

}  // namespace raubase::motor