#ifndef RAUBASE_CONTROL_INTERFACE
#define RAUBASE_CONTROL_INTERFACE

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

#include "common/utils/types.hpp"

namespace raubase::math {

/**
 * @brief Class defining a controller.
 */
template <typename R, typename Y>
struct ControllerInterface {
  typedef sptr<ControllerInterface<R, Y>> SharedPtr;

  /**
   * @brief Reset the internal state of the controller.
   *
   */
  virtual void reset_history() = 0;

  /**
   * @brief Update the controller (new time tick) with the reference and the measure.
   * @param dt the time since the last update (in seconds)
   * @param ref the reference to follow
   * @param measured the measured output from last tick
   * @param saturation if the controller is at its limit or not
   */
  virtual Y update(double dt, const R& ref, const Y& measured, bool saturation) = 0;

  /**
   * @brief Set whether the angle should be folder after computing the error.
   */
  void set_fold_angle(bool _fold) { fold_angle = _fold; }

 protected:
  bool fold_angle = false;
};

// Way of changing controllers floating precision
typedef double control_val;

typedef ControllerInterface<control_val, control_val> ControlInterface;

}  // namespace raubase::math

#endif