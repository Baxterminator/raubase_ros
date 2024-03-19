#include "common/math/pid_control.hpp"

#include "common/math/math.hpp"

namespace raubase::math {

PILeadController::PILeadController(control_val Kp, control_val td, control_val alpha_d,
                                   control_val ti)
    : kp(Kp),
      use_integrator(ti > MIN_TAU),
      use_lead(td > MIN_TAU),
      le0((use_lead) ? 2 * td : 0),
      lu0((use_lead) ? le0 * alpha_d : 0),
      ie((use_integrator) ? 1 / (2 * ti) : 0.0) {
  reset_history();
}

void PILeadController::reset_history() {
  ep = 0;
  last_ep = 0;
  up = 0;
  last_up = 0;
  ui = 0;
  last_ui = 0;
}

control_val PILeadController::update(double dt, const control_val &ref, const control_val &measured,
                                     bool saturation) {
  if (dt < 1E-8) return last_ui + last_up;
  if (fold_angle)
    ep = kp * math::natural_angle(ref - measured);
  else
    ep = kp * (ref - measured);
  if (use_lead)
    up = update_lead(dt, ep);
  else
    up = ep;

  if (use_integrator && !saturation)
    ui = update_integrator(dt, up);
  else
    ui = last_ui;

  // Update internal registers
  last_ep = ep;
  last_up = up;
  last_ui = ui;

  return ui + up;
}

control_val PILeadController::update_lead(double dt, control_val new_ep) {
  dtlu0_inv = 1 / (dt + lu0);
  return ((dt + le0) * dtlu0_inv * ep) + ((dt - le0) * dtlu0_inv * last_ep) -
         ((dt - lu0) * dtlu0_inv * last_up);
}

control_val PILeadController::update_integrator(double dt, control_val new_ep) const {
  return (up + last_up) * dt * ie + last_ui;
}

}  // namespace raubase::math