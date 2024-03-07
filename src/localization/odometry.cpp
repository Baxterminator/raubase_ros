#include <cmath>

#include "common/math.hpp"
#include "localization/localization.hpp"

namespace raubase::loc {

Odometry::Odometry(NodeOptions opts) : Node(NODE_NAME, opts) {
  RCLCPP_INFO(get_logger(), "Initializing odometry");

  // Declare parameters
  gear = declare_parameter("gear_ratio", DEF_GEAR_RATIO);
  wheel_d = declare_parameter("wheel_diameters_m", DEF_WHEEL_D);
  tick_per_rev = declare_parameter("tick_per_rev", DEF_TICK_PER_REV);
  base = declare_parameter("base_width_m", DEF_BASE);
  dist_per_tick = (wheel_d * M_PI) / gear / tick_per_rev;

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

  // Compute time between messages
  double dt = (last_enc->stamp.sec - last_enc_used->stamp.sec) +
              (last_enc->stamp.nanosec - last_enc_used->stamp.nanosec) * 1E-9;

  RCLCPP_INFO(get_logger(), "Computing new values");
  computeNewWheelVelocities(dt);
  computeNewWorldPosition(dt);

  // Publish new odometry
  RCLCPP_INFO(get_logger(), "Publishing results");
  odom_pub->publish(odom_msg);
  last_enc_used = last_enc;
  last_enc_has_been_used = true;
}

void Odometry::computeNewWheelVelocities(double dt) {
  // Compute change in ticks
  double denc_right = math::saturate(last_enc->right - last_enc_used->right, MAX_TICK_CHANGE);
  double denc_left = math::saturate(last_enc->left - last_enc_used->left, MAX_TICK_CHANGE);

  // Compute wheel velocity
  odom_msg.v_right = denc_right * dist_per_tick / dt;
  odom_msg.v_left = denc_left * dist_per_tick / dt;
}

void Odometry::computeNewWorldPosition(double dt) {
  // Compute 2D linear & half of the angular displacement
  odom_msg.v_lin = (odom_msg.v_left + odom_msg.v_right) / 2.0;
  odom_msg.turn_rate = (odom_msg.v_left - odom_msg.v_right) / base / 2.0;

  // Integrate for getting positions and heading.
  // Compute the position displacement with half of the angular displacement to
  // get an average between the two positions.
  odom_msg.heading += odom_msg.turn_rate;
  odom_msg.x += std::cos(odom_msg.heading) * odom_msg.turn_rate;
  odom_msg.y += std::sin(odom_msg.heading) * odom_msg.turn_rate;

  // Complete the angular displacement with the other half
  odom_msg.heading = math::natural_angle(odom_msg.heading + odom_msg.turn_rate);

  // Compute linear & angular velocities
  odom_msg.v_lin /= dt;
  odom_msg.turn_rate /= dt;
}

}  // namespace raubase::loc

int main(int argc, char** argv) {
  init(argc, argv);

  auto node = std::make_shared<raubase::loc::Odometry>(NodeOptions{});

  while (ok()) spin_some(node);

  shutdown();
  return 0;
}