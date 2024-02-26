#include "teensy/teensy.hpp"

#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <queue>
#include <ratio>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>

namespace raubase::teensy {

///////////////////////////////////////////////////////////////////////////////

Teensy::Teensy(rclcpp::NodeOptions opts) : rclcpp::Node(Teensy::NODE_NAME, opts) {
  RCLCPP_INFO(get_logger(), "Initializing the node parameters!");
  // ---------------------------- Configurations ------------------------------
  _bot_name = declare_parameter("name", "robotbot");
  _confirm_timeout = declare_parameter("ack_timeout_s", 0.04);
  _connect_timeout = declare_parameter("con_timeout_s", 20.0);
  _activity_timeout = declare_parameter("act_timeout_s", 10.0);
  _max_resend_cnt = declare_parameter("max_resend_num", 50);
  _timer_period = std::chrono::microseconds(declare_parameter("loop_rate_us", 200));
  _timer_wait = std::chrono::microseconds(declare_parameter("loop_wait_us", 1000));
  usb_co.name = declare_parameter("usb_device", "/dev/ttyACM0");

  _regbotNumber = declare_parameter("regbot_number", -1);
  _regbotHardware = declare_parameter("regbot_hardware", -1);

  // ---------------------------- Verifications -------------------------------
  if (_confirm_timeout < 0.01) _confirm_timeout = 0.02;

  // -------------------------- Setup Proxies List -----------------------------
  SendingCallback _sending_cbk = [this](sptr<MSG> msg, bool direct) { this->send(msg, direct); };
  converters = {
      {proxy::HeartBeatProxy::TEENSY_MSG,
       TeensyProxy::make_shared<proxy::HeartBeatProxy>(_sending_cbk)},
      {proxy::EncoderProxy::TEENSY_MSG,
       TeensyProxy::make_shared<proxy::EncoderProxy>(_sending_cbk)},

  };

  // -------------------------- Communication Init ----------------------------
  trx_runtime = create_wall_timer(_timer_period, std::bind(&Teensy::TRXLoop, this));
}

void Teensy::setupTeensy() {
  RCLCPP_INFO(get_logger(), "Sending to the queue the Teensy configuration messages");

  // Launch initialization phase
  send(std::make_shared<MSG>(INIT_MSG));
  char s[MSG::MBL];

  // Regbot number
  if (_regbotNumber >= 0) {
    snprintf(s, MSG::MBL, "%s %d", REG_NUM_MSG, _regbotNumber);
    send(MSG::make(s));
  }

  // Regbot hardware
  if (_regbotHardware > 0) {
    snprintf(s, MSG::MBL, "%s %d", REG_HW_MSG, _regbotHardware);
    send(MSG::make(s));
  }

  // Robot name
  snprintf(s, MSG::MBL, "%s %s", BOT_NAME_MSG, _bot_name.c_str());
  send(MSG::make(s));

  // Flash
  send(MSG::make(FLASH_MSG));
}

void Teensy::setupProxiesROS() {
  RCLCPP_INFO(get_logger(), "Initializing the messages proxies (ROS)!");
  // --------------------------- Messages converters --------------------------
  for (auto& [key, val] : this->converters) {
    val->setupParams(this->shared_from_this());
  }
}

void Teensy::setupProxiesTeensy() {
  RCLCPP_INFO(get_logger(), "Initializing the messages proxies (Teensy)!");
  // --------------------------- Messages converters --------------------------
  for (auto& [key, val] : this->converters) {
    val->setupSubscriptions();
  }
}

///////////////////////////////////////////////////////////////////////////////

void Teensy::stopTeensy() {
  RCLCPP_INFO(get_logger(), "Stopping communication with the Teensy Board!");
  sendDirect(std::make_shared<MSG>("leave"));
  sendDirect(std::make_shared<MSG>("disp stopped"));
}

Teensy::~Teensy() {
  stopTeensy();
  usleep(1000000);  // Waiting for everything to be sent (1s)
  closeUSB();
}

}  // namespace raubase::teensy

///////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto teensy = std::make_shared<raubase::teensy::Teensy>(rclcpp::NodeOptions{});
  teensy->setupProxiesROS();
  while (rclcpp::ok()) rclcpp::spin_some(teensy);
  rclcpp::shutdown();
}