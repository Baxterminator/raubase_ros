#include <cmath>
#include <rclcpp/logging.hpp>

#include "common/math/math.hpp"
#include "common/robot/kinematics.hpp"
#include "common/utils/types.hpp"
#include "localization/localization.hpp"

using raubase::kinematics::Wheel;

namespace raubase::loc {

Odometry::Odometry(NodeOptions opts) : Node(NODE_NAME, opts) {
  RCLCPP_INFO(get_logger(), "Initializing odometry");

  // Declare parameters
  odom_msg.turn_rate = 0;
  odom_msg.v_lin = 0;
  odom_msg.v_left = 0;
  odom_msg.v_right = 0;
  odom_msg.heading = 0;
  odom_msg.x = 0;
  odom_msg.y = 0;

  robot = TwoWheeledRoverKinematics::make(
      Wheel(declare_parameter(Params::R_WHEEL_D, Default::WHEEL_D),
            declare_parameter(Params::R_WHEEL_RATIO, Default::GEAR_RATIO),
            declare_parameter(Params::R_WHEEL_TPR, Default::TICK_PER_REV)),
      Wheel(declare_parameter(Params::L_WHEEl_D, Default::WHEEL_D),
            declare_parameter(Params::L_WHEEL_RATIO, Default::GEAR_RATIO),
            declare_parameter(Params::L_WHEEL_TPR, Default::TICK_PER_REV)),
      MAX_TICK_CHANGE, declare_parameter(Params::BASE_WIDTH, Default::DEF_BASE),
      declare_parameter(Params::F_LAT, Default::F_LAT),
      declare_parameter(Params::F_LON, Default::F_LON));

  //< Whether the node should run in "consuming" (i.e. when receiving a message) or
  // in "frequency" (i.e. run x times a second)
  RCLCPP_INFO(get_logger(), "Initializing pub/sub");
  auto freq = declare_parameter(Params::ODOM_FREQ, Default::ODOM_FREQ);
  if (freq == -1) {
    encoder_sub = create_subscription<msg::DataEncoder>(
        Topics::SUB_ENC, 10, [this](const msg::DataEncoder::SharedPtr msg) {
          last_enc = msg;
          last_enc_has_been_used = false;
          updateOdometry();
        });
  } else {
    encoder_sub = create_subscription<msg::DataEncoder>(
        Topics::SUB_ENC, 10, [this](const msg::DataEncoder::SharedPtr msg) {
          last_enc = msg;
          last_enc_has_been_used = false;
        });
    odom_loop = create_wall_timer(microseconds((long)(1E6 * 1 / freq)),
                                  std::bind(&Odometry::updateOdometry, this));
  }
  odom_pub = create_publisher<msg::ResultOdometry>(Topics::PUB_ODOM, QOS);
}

///////////////////////////////////////////////////////////////////////////////

void Odometry::updateOdometry() {
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), THROTTLE_DUR, "Odometry loop");

  // Initialize last enc
  if (last_enc_used == nullptr) {
    last_enc_used = last_enc;
    return;
  }

  // Sanity check for last_enc value
  if (last_enc == nullptr) return;

  // Prevent from reusing the same one
  if (last_enc_has_been_used) return;

  // Update Odometry
  RCLCPP_INFO(get_logger(), "Computing new values");
  robot->updateOdometry(last_enc_used, last_enc, odom_msg);
  odom_msg.stamp = get_clock()->now();

  // Publish new odometry
  RCLCPP_INFO(get_logger(), "Publishing results");
  odom_pub->publish(odom_msg);
  last_enc_used = last_enc;
  last_enc_has_been_used = true;
}

}  // namespace raubase::loc

int main(int argc, char** argv) {
  init(argc, argv);

  auto node = std::make_shared<raubase::loc::Odometry>(NodeOptions{});

  while (ok()) spin_some(node);

  shutdown();
  return 0;
}