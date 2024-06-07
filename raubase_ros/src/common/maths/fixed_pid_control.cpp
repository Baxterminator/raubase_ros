#include "common/math/fixed_pid_control.hpp"

#include "common/math/math.hpp"

namespace raubase::math {

FixedPILeadController::FixedPILeadController(control_val Kp, control_val td, control_val alpha_d,
                                             control_val ti, control_val sampling_time)
    : kp(Kp),
      use_integrator(ti > MIN_TAU),
      use_lead(td > MIN_TAU),
      Kd1((use_lead) ? (sampling_time + 2 * td) / (sampling_time + 2 * td * alpha_d) : 0),
      Kd2((use_lead) ? (sampling_time - 2 * td) / (sampling_time + 2 * td * alpha_d) : 0),
      Kd3((use_lead) ? (sampling_time - 2 * td * alpha_d) / (sampling_time + 2 * td * alpha_d) : 0),
      Ki((use_integrator) ? sampling_time / (2 * ti) : 0.0) {
  reset_history();
}

void FixedPILeadController::reset_history() {
  ep = 0;
  last_ep = 0;
  up = 0;
  last_up = 0;
  ui = 0;
  last_ui = 0;
}

control_val FixedPILeadController::update(control_val dt, const control_val &ref,
                                          const control_val &measured, bool saturation) {
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

control_val FixedPILeadController::update_lead([[maybe_unused]] control_val dt,
                                               control_val new_ep) {
  return Kd1 * ep + Kd2 * last_ep - Kd3 * last_up;
}

control_val FixedPILeadController::update_integrator([[maybe_unused]] control_val dt,
                                                     control_val new_ep) const {
  return (up + last_up) * Ki + last_ui;
}

}  // namespace raubase::math