#include "teensy/teensy.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <queue>
#include <ratio>
#include <rclcpp/executors.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>

namespace raubase::teensy {

///////////////////////////////////////////////////////////////////////////////

Teensy::Teensy(rclcpp::NodeOptions opts) : rclcpp::Node(Teensy::NODE_NAME, opts) {
  // ---------------------------- Configurations ------------------------------
  _device = declare_parameter("name", "robotbot");
  _confirm_timeout = declare_parameter("ack_timeout_s", 0.04);
  _connect_timeout = declare_parameter("con_timeout_s", 20.0);
  _activity_timeout = declare_parameter("act_timeout_s", 10.0);
  _timer_period = std::chrono::microseconds(declare_parameter("loop_rate_us", 200));
  _timer_wait = std::chrono::microseconds(declare_parameter("loop_wait_us", 1000));
  usb_co.name = declare_parameter("usb_device", "/dev/ttyACM0");

  // ---------------------------- Verifications -------------------------------
  if (_confirm_timeout < 0.01) _confirm_timeout = 0.02;

  // -------------------------- Communication Init ----------------------------
  trx_runtime = create_wall_timer(_timer_period, std::bind(&Teensy::TRXLoop, this));
}

void Teensy::setupProxy() {
  // --------------------------- Messages converters --------------------------
  for (auto& [key, val] : this->converters) {
    val->setup(this->shared_from_this());
  }
}

Teensy::~Teensy() { closeUSB(); }

}  // namespace raubase::teensy

///////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto teensy = std::make_shared<raubase::teensy::Teensy>(rclcpp::NodeOptions{});
  teensy->setupProxy();
  while (rclcpp::ok()) rclcpp::spin_some(teensy);
  rclcpp::shutdown();
}