#include "teensy/teensy.hpp"

#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <queue>
#include <ratio>

#include "teensy/interface/proxy_interface.hpp"
#include "teensy/proxy/distance.hpp"
#include "teensy/proxy/encoder.hpp"
#include "teensy/proxy/heartbeat.hpp"
#include "teensy/proxy/imu.hpp"
#include "teensy/proxy/line_sensor.hpp"
#include "teensy/proxy/motor.hpp"
#include "teensy/proxy/servo.hpp"

namespace raubase::teensy {

///////////////////////////////////////////////////////////////////////////////

Teensy::Teensy(rclcpp::NodeOptions opts) : rclcpp::Node(Teensy::NODE_NAME, opts) {
  RCLCPP_INFO(get_logger(), "Initializing the node parameters!");
  // ---------------------------- Configurations ------------------------------
  _bot_name = declare_parameter("name", "robobot");
  _confirm_timeout = declare_parameter("ack_timeout_s", 0.04);
  _connect_timeout = declare_parameter("con_timeout_s", 20.0);
  _activity_timeout = declare_parameter("act_timeout_s", 10.0);
  _max_resend_cnt = declare_parameter("max_resend_num", 50);
  _timer_period = microseconds(declare_parameter("loop_rate_us", 50));
  _timer_wait = microseconds(declare_parameter("loop_wait_us", 1000));
  usb_co.name = declare_parameter("usb_device", "/dev/ttyACM0");

  _regbotNumber = declare_parameter("regbot_number", -1);
  _regbotHardware = declare_parameter("regbot_hardware", -1);

  // ---------------------------- Verifications --------------------------Look at your class
  // definition. Find the first non-inline virtual function that is not pure virtual (not "= 0") and
  // whose definition you provide (not "= default"). -----
  if (_confirm_timeout < 0.01) _confirm_timeout = 0.02;

  // -------------------------- Setup Proxies List -----------------------------
  SendingCallback _sending_cbk = [this](sptr<MSG> msg, bool direct) { this->send(msg, direct); };
  add_proxy(TeensyProxy::make_shared<proxy::HeartBeatProxy>(_sending_cbk),
            proxy::HeartBeatProxy::TEENSY_MSG);
  add_proxy(TeensyProxy::make_shared<proxy::EncoderProxy>(_sending_cbk),
            proxy::EncoderProxy::TEENSY_MSG);
  add_proxy(TeensyProxy::make_shared<proxy::LineSensorProxy>(_sending_cbk),
            proxy::LineSensorProxy::TEENSY_MSG);
  add_proxy(TeensyProxy::make_shared<proxy::IMUProxy>(_sending_cbk),
            {proxy::IMUProxy::GYRO_MSG, proxy::IMUProxy::ACC_MSG});
  add_proxy(TeensyProxy::make_shared<proxy::DistanceProxy>(_sending_cbk),
            proxy::DistanceProxy::TEENSY_MSG);
  add_proxy(TeensyProxy::make_shared<proxy::MotorProxy>(_sending_cbk));
  add_proxy(proxy::ServoProxy::make(_sending_cbk), proxy::ServoProxy::TEENSY_MSG);

  // -------------------------- Communication Init ----------------------------
  trx_runtime = create_wall_timer(_timer_period, std::bind(&Teensy::TRXLoop, this));
}

void Teensy::setupTeensy() {
  RCLCPP_INFO(get_logger(), "Sending to the queue the Teensy configuration messages");

  // Launch initialization phase
  send(std::make_shared<MSG>(INIT_MSG, false));
  char s[MSG::MBL];

  // Regbot number
  if (_regbotNumber >= 0) {
    snprintf(s, MSG::MBL, "%s %d", REG_NUM_MSG, _regbotNumber);
    send(MSG::make(s, false));
  }

  // Regbot hardware
  if (_regbotHardware > 0) {
    snprintf(s, MSG::MBL, "%s %d", REG_HW_MSG, _regbotHardware);
    send(MSG::make(s, false));
  }

  // Robot name
  snprintf(s, MSG::MBL, "%s %s", BOT_NAME_MSG, _bot_name.c_str());
  send(MSG::make(s, false));

  // Flash
  send(MSG::make(FLASH_MSG, false));
}

void Teensy::setupProxiesROS() {
  RCLCPP_INFO(get_logger(), "Initializing the messages proxies (ROS)!");
  for (auto& p : this->_proxies) {
    p->setupParams(this->shared_from_this());
  }
}

void Teensy::setupProxiesTeensy() {
  RCLCPP_INFO(get_logger(), "Initializing the messages proxies (Teensy)!");
  for (auto& p : this->_proxies) {
    p->setupSubscriptions();
  }
}

///////////////////////////////////////////////////////////////////////////////

void Teensy::stopProxiesTeensy() {
  RCLCPP_INFO(get_logger(), "Launching proxies cleanup sequence (Teensy)!");
  for (auto& p : this->_proxies) {
    p->closeTeensy();
  }
}

void Teensy::stopTeensy() {
  RCLCPP_INFO(get_logger(), "Stopping communication with the Teensy Board!");
  sendDirect(std::make_shared<MSG>("leave", false));
  sendDirect(std::make_shared<MSG>("disp stopped", false));
}

Teensy::~Teensy() {
  stopProxiesTeensy();
  usleep(1000000);  // Waiting for everything to be sent (1s)
  stopTeensy();
  usleep(1000000);  // Waiting for everything to be sent (1s)
  closeUSB();
}

///////////////////////////////////////////////////////////////////////////////

void Teensy::add_proxy(TeensyProxy::SharedPtr prox, const std::vector<const char*>& msgs) {
  // Add proxy to the list
  _proxies.push_back(prox);
  unsigned long proxy_idx = _proxies.size() - 1;

  // Add mapping
  for (auto& m : msgs) {
    _proxies_mapping.insert_or_assign(m, proxy_idx);
  }
}

void Teensy::add_proxy(TeensyProxy::SharedPtr prox, const char* const msg) {
  // Add proxy to the list
  _proxies.push_back(prox);
  unsigned long proxy_idx = _proxies.size() - 1;

  // Add mapping
  _proxies_mapping.insert_or_assign(msg, proxy_idx);
}

void Teensy::add_proxy(TeensyProxy::SharedPtr prox) {
  // Add proxy to the list
  _proxies.push_back(prox);
}

}  // namespace raubase::teensy

///////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto teensy = std::make_shared<raubase::teensy::Teensy>(rclcpp::NodeOptions{});
  teensy->setupProxiesROS();
  while (rclcpp::ok()) {
    rclcpp::spin(teensy);
  }
  rclcpp::shutdown();
}