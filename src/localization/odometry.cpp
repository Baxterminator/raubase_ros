#include <cmath>

#include "common/robot/kinematics.hpp"
#include "common/utils/math.hpp"
#include "localization/localization.hpp"

namespace raubase::loc {

Odometry::Odometry(NodeOptions opts) : Node(NODE_NAME, opts) {
  RCLCPP_INFO(get_logger(), "Initializing odometry");

  odom_msg.turn_rate = 0;
  odom_msg.v_lin = 0;
  odom_msg.v_left = 0;
  odom_msg.v_right = 0;
  odom_msg.heading = 0;
  odom_msg.x = 0;
  odom_msg.y = 0;

  robot = TwoWheeledRoverKinematics::make(
      Wheel(declare_parameter("right_wheel_diameter_m", DEF_WHEEL_D),
            declare_parameter("right_gear_ratio", DEF_GEAR_RATIO),
            declare_parameter("right_tick_per_rev", DEF_TICK_PER_REV)),
      Wheel(declare_parameter("left_wheel_diameter_m", DEF_WHEEL_D),
            declare_parameter("left_gear_ratio", DEF_GEAR_RATIO),
            declare_parameter("left_tick_per_rev", DEF_TICK_PER_REV)),
      MAX_TICK_CHANGE, declare_parameter("base_width_m", DEF_BASE));

  // Declare parameters
  odom_loop_period = microseconds((long)(1E6 / declare_parameter("odom_freq", DEF_ODOM_FREQ)));

  // Declare ROS participants
  RCLCPP_INFO(get_logger(), "Initializing pub/sub");
  encoder_sub = create_subscription<msg::EncoderState>(
      SUB_ENC_TOPIC, 10, [this](const msg::EncoderState::SharedPtr msg) {
        last_enc = msg;
        last_enc_has_been_used = false;
      });
  odom_pub = create_publisher<msg::Odometry>(PUB_ODOM_TOPIC, QOS);
  odom_loop = create_wall_timer(odom_loop_period, std::bind(&Odometry::updateOdometry, this));
}

///////////////////////////////////////////////////////////////////////////////

void Odometry::updateOdometry() {
  RCLCPP_INFO(get_logger(), "Odometry loop");
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