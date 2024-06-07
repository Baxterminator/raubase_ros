#ifndef RAUBASE_PID_CONTROLLER
#define RAUBASE_PID_CONTROLLER

/*
Copyright (C) 2017-2024 by DTU
Authors:
  Christan Andersen: jcan@dtu.dk
  Geoffrey Côte: geoffrey.cote@centraliens-nantes.org

The MIT License (MIT)  https://mit-license.org/

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the “Software”), to deal in the Software without
restriction, including without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies
or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include "common/math/controller.hpp"
#include "common/utils/types.hpp"

namespace raubase::math {

/**
 * @brief Implementation of a PID controller for input and output of dimension 1.
 * @author Christan Andersen: jcan@dtu.dk
 * @author Geoffrey Côte: geoffrey.cote@centraliens-nantes.org
 * @since 1.0.0
 *
 * The PI-Lead controller is made of three parts: a potential gain, a lead part and an integral
 * part. The continuous transfer function is:
 *           u(s)       (td*s + 1)          1
 *    G(s) = ---- = Kp ------------- (1 + ------)
 *           e(s)      (ad*td*s + 1)       ti*s
 *
 * In this implementation we compute the value from left to right order:
 *    - 1. Potential gain
 *    - 2. Lead part
 *    - 3. Integrator part
 */
struct PILeadController : public ControllerInterface<control_val, control_val> {
  constval float MIN_TAU{1e-3};

  // ================================================ ==========================
  //                                 LifeCycle
  // ==========================================================================
 public:
  /**
   * @brief Construct a PI-Lead controller.
   *
   * @param Kp the potential gain of the controller
   * @param td the lead constant time (in s)
   * @param alpha_d the least width (in [0,1])
   * @param ti the integrator constant time
   */
  PILeadController(control_val Kp, control_val td, control_val alpha_d, control_val ti);

  ~PILeadController() override {}

  /**
   * @brief Construct a PI-Lead controller.
   *
   * @param Kp the potential gain of the controller
   * @param td the lead constant time (in s)
   * @param alpha_d the least width (in [0,1])
   * @param ti the integrator constant time
   * @return a shared pointer of the controller
   */
  static sptr<PILeadController> make(control_val Kp, control_val td, control_val alpha_d,
                                     control_val ti) {
    return std::make_shared<PILeadController>(Kp, td, alpha_d, ti);
  }

  /**
   * @brief Reset the internal state of the controller.
   */
  void reset_history() override;

  /**
   * @brief Update the controller (new time tick) with the reference and the measure.
   *
   * @param dt the time since the last update (in seconds)
   * @param ref the reference to follow
   * @param measured the measured output from last tick
   * @param saturation if the controller is at its limit or not
   */
  control_val update(control_val dt, const control_val& ref, const control_val& measured,
                     bool saturation) override;

 private:
  /**
   * @brief Update the lead part of the controller.
   *
   * The Lead filter consists of a pole - zero 1st order system.
   * The continuous transfer function is:
   *                 (td*s + 1)
   *    G_lead(s) = -------------
   *                (ad*td*s + 1)
   *
   * When translating to z (Tustin), we use:
   *        2(z - 1)
   *    s = ---------
   *        T(z + 1)
   *
   * Hence we get the following discret transfer
   *                 u(z)       (T + 2 td) + (T - 2 td)z^-1
   *    G_lead(z) = ------ = ----------------------------------
   *                 e(z)    (T + 2 td ad) + (T - 2 td ad) z^-1
   *
   * Then we have:
   *  u(n)*(T + 2*td*ad) = e(n)*(T + 2*td) + e(n-1)*(T - 2*td) - u(n-1)*(T - 2*td*ad)
   *
   * We can precompute some of the constants to get:
   *               T + le0            T - le0            T - lu0
   *  u(n) = e(n) --------- + e(n-1) --------- - u(n-1) ---------
   *               T + lu0            T + lu0            T + lu0
   *
   * With the constants:
   *  - le0 = 2*td
   *  - lu0 = 2*td*ad
   *
   * @param dt the time since the last tick
   * @param new_ep the value of the error times the potential gain for this tick.
   * @returns the output value of the lead part for this tick
   */
  control_val update_lead(control_val dt, control_val new_ep);

  /**
   * @brief Update the integrator part of the controller.
   *
   * For the integrator part, we use a basic integrator:
   *               1
   *    G_i(s) = ------
   *              ti*s
   *
   *  When translating to z (Tustin method), we use:
   *        2(z - 1)
   *    s = ---------
   *        T(z + 1)
   *
   * Thus obtaining the discret transfer:
   *               T * (1 + z^-1)
   *  G_i(z) = -------------------
   *            2*ti * (1 - z^-1)
   *
   * And the output value is:
   *  u(n)*2*ti = T * (e(n) + e(n-1)) + 2*ti*u(n-1)
   *
   * We can precompute one constant to get:
   *
   *  u(n) = [e(n) + e(n-1)] T*ie + u(n-1)
   *
   * With ie = 1/(2*ti)
   *
   * @param dt the time since the last tick
   * @param new_up the value of the output of the lead part for this tick
   * @returns the output value of the integrator part for this tick
   */
  control_val update_integrator(control_val dt, control_val new_up) const;

  // ==========================================================================
  //                                 Members
  // ==========================================================================
 private:
  // PI-Lead parameters
  const control_val kp;  //< Potential gain of the controller

  // Precompute values
  const bool use_integrator;  //< Whether we want to compute the integration part
  const bool use_lead;        //< Whether we want to use the lead part
  const control_val le0;      //< Numerator constant in lead controller
  const control_val lu0;      //< Denominator constant in lead controller
  const control_val ie;       //< Constant for integrator controller

  // Controller state
  control_val ep, last_ep;  //< Errors times the potential gain for both actual and last ticks
  control_val up, last_up;  //< Lead output for both actual and last ticks
  control_val ui, last_ui;  //< Integrator output for both actual and last ticks
  control_val dtlu0_inv;    //< Computation for reducing lead operation cost
};

}  // namespace raubase::math

#endif