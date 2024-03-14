#ifndef RAUBASE_KINEMATICS
#define RAUBASE_KINEMATICS

/*
Copyright (C) 2017-2024 by DTU
Authors:
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

#include <memory>
#include <raubase_msgs/msg/data_encoder.hpp>
#include <raubase_msgs/msg/result_odometry.hpp>

#include "common/utils/math.hpp"
#include "common/utils/types.hpp"

using raubase_msgs::msg::DataEncoder;
using raubase_msgs::msg::ResultOdometry;

namespace raubase::kinematics {

/**
 * @brief Structure representing a wheel of the robot.
 */
struct Wheel {
  double diameter;
  double gear_ratio;
  int tick_per_rev;
  double dist_per_tick;

  /**
   * @brief Constructing a Wheel parameters objects
   *
   * @param d the diameter of the wheel
   * @param gr the gear ratio of the reducer
   * @param tpr the number of tick per revolutions
   */
  Wheel(double d, double gr, int tpr)
      : diameter(d), gear_ratio(gr), tick_per_rev(tpr), dist_per_tick(d * M_PI / gr / tpr) {}
};

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Kinematics of a rover with two motorized wheels on the same axle.
 */
struct TwoWheeledRoverKinematics {
  typedef sptr<TwoWheeledRoverKinematics> SharedPtr;

  TwoWheeledRoverKinematics(Wheel _right_wheel, Wheel _left_wheel, double _max_tick_change,
                            double base_width)
      : max_tick_change(_max_tick_change),
        base(base_width),
        left(_left_wheel),
        right(_right_wheel) {}

  static TwoWheeledRoverKinematics::SharedPtr make(const Wheel &_right_wheel,
                                                   const Wheel &_left_wheel,
                                                   double _max_tick_change, double base_width) {
    return std::make_shared<TwoWheeledRoverKinematics>(_right_wheel, _left_wheel, _max_tick_change,
                                                       base_width);
  }

  // ==========================================================================
  //                                    Encoders
  // ==========================================================================

  /**
   * @brief Compute the distance made by the right wheel during the delta encoders values.
   *
   * @param delta the difference in encoders values.
   */
  inline double enc2RWdist(const int delta) const { return enc2Wdist(delta, right.dist_per_tick); }

  /**
   * @brief Compute the distance made by the right wheel during the delta encoders values.
   *
   * @param delta the difference in encoders values.
   */
  inline double enc2LWdist(const int delta) const { return enc2Wdist(delta, left.dist_per_tick); }

  /**
   * @brief Return the encoder value associated to the given right wheel position.
   *
   * @param wheel_pos the position in radians
   */
  inline int RW_pos2enc(const double wheel_pos) {
    return wheel_pos / math::M_2PI * right.tick_per_rev;
  }

  /**
   * @brief Return the encoder value associated to the given left wheel position.
   *
   * @param wheel_pos the position in radians*2*np.pi/self.tick_per_rev
   */
  inline int LW_pos2enc(const double wheel_pos) {
    return wheel_pos / math::M_2PI * left.tick_per_rev;
  }

  // ==========================================================================
  //                        Linear & Angular  Displacements
  // ==========================================================================
  /**
   * @brief Compute the linear displacement based on the difference of encoders values on both
   * the right and left wheels. The displacement is computed as the mean of both displacement.
   *
   * @param right the distance made on the right wheel
   * @param left the distance made on the left wheel
   * @return the linear displacement made by the robot
   */
  inline double rl_disp2lin_disp(const double right, const double left) const {
    return (right + left) / 2.0;
  };

  /**
   * @brief Compute the angular displacement based on the difference of movement on both right and
   * left wheels.
   *
   * @param right the distance made on the right wheel
   * @param left the distance made on the left wheel
   * @param half true if the function should return half of the value
   * @return the angular displacement made by the robot
   */
  inline double rl_disp2ang_disp(const double right, const double left,
                                 const bool half = false) const {
    if (half) return (right - left) / base / 2;
    return (right - left) / base;
  }

  // ==========================================================================
  //                                Odometry
  // ==========================================================================
  /**
   * @brief Compute and apply the odometry result from the linear and angular displacements.
   *
   * @param v the linear displacement of the robot
   * @param w half of the angular displacement of the robot
   * @param x the x position (value will be modified)
   * @param y the y position (value will be modified)
   * @param h the heading (value will be modified)
   */
  inline void pos_odometry(const double v, const double w, double &x, double &y, double &h) const {
    h += w;
    x += std::cos(h) * v;
    y += std::sin(h) * v;
    h = math::natural_angle(h + w);
  }

  // ==========================================================================
  //                                Wrapper
  // ==========================================================================
  /**
   * @brief Wrapper for the whole odometry computation.
   *
   * @param last_used the last used encoder state
   * @param newest the newest encoder state
   * @param odom_msg the last odometry message
   */
  inline void updateOdometry(const DataEncoder::SharedPtr last_used,
                             const DataEncoder::SharedPtr newest, ResultOdometry &odom_msg) const {
    double dt = math::timeBtwStamps(last_used->stamp, newest->stamp);

    // Compute wheels displacement
    odom_msg.v_right = enc2RWdist(newest->right - last_used->right);
    odom_msg.v_left = enc2RWdist(newest->left - last_used->left);

    // Compute robot displacement
    odom_msg.v_lin = rl_disp2lin_disp(odom_msg.v_right, odom_msg.v_left);
    odom_msg.turn_rate = rl_disp2ang_disp(odom_msg.v_right, odom_msg.v_left, true);

    pos_odometry(odom_msg.v_lin, odom_msg.turn_rate, odom_msg.x, odom_msg.y, odom_msg.heading);

    // Convert to velocities
    odom_msg.v_lin = ((dt > 0) ? odom_msg.v_lin / dt : 0);
    odom_msg.turn_rate = ((dt > 0) ? odom_msg.turn_rate / dt : 0);
    odom_msg.v_right = ((dt > 0) ? odom_msg.v_right / dt : 0);
    odom_msg.v_left = ((dt > 0) ? odom_msg.v_left / dt : 0);
  }

  // ==========================================================================
  //                       Private computation methods
  // ==========================================================================
 private:
  inline double enc2Wdist(const int delta, const double dist_per_tick) const {
    return math::cst_outside(delta, max_tick_change) * dist_per_tick;
  }

 private:
  int max_tick_change;
  double base;
  Wheel left, right;
};
}  // namespace raubase::kinematics

#endif