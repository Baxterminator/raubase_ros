#include "teensy/proxy/line_sensor.hpp"

#include <rclcpp/logging.hpp>
#include <robotbot_msgs/msg/detail/line_sensor_data__struct.hpp>
#include <robotbot_msgs/srv/detail/toggle_line_sensor__struct.hpp>
#include <sstream>

#include "teensy/interface/message.hpp"

namespace raubase::teensy::proxy {

void LineSensorProxy::setupParams(rclcpp::Node::SharedPtr node) {
  RCLCPP_INFO(logger, "Initializing proxy %s", NODE_NAME);

  // Declaring parameters for the encoder
  _refresh_rate = node->declare_parameter("liv_ms", 8);

  // Initializing working components
  publisher = node->create_publisher<LineSensorData>(PUBLISHING_TOPIC, QOS);
  setting_srv = node->create_service<SetLineSensor>(
      SETTING_SERVICE, std::bind(&LineSensorProxy::setLineSensorMode, this, std::placeholders::_1,
                                 std::placeholders::_2));
  toggling_srv = node->create_service<ToggleLineSensor>(
      TOGGLE_SERVICE, std::bind(&LineSensorProxy::setSensorOnOff, this, std::placeholders::_1,
                                std::placeholders::_2));
  clock = node->get_clock();

  // Initializing internal state
  _internal_req = std::make_shared<SetLineSensor::Request>();
  _internal_req->on = node->declare_parameter("liv_def_on", false);
  _internal_req->high_power = node->declare_parameter("liv_def_power", true);
}

void LineSensorProxy::setupSubscriptions() {
  RCLCPP_INFO(logger, "Initializing proxy %s", NODE_NAME);
  subscribeTeensyComponent(TEENSY_COMP, _refresh_rate);

  // Setup line sensor
  setLineSensorMode(_internal_req);
}

void LineSensorProxy::closeTeensy() {
  RCLCPP_INFO(logger, "Cleaning-up proxy %s", NODE_NAME);
  // Stop the line sensor on closing the proxy
  _internal_req->on = false;
  _internal_req->high_power = false;
  setLineSensorMode(_internal_req);
}

void LineSensorProxy::setLineSensorMode(const SetLineSensor::Request::SharedPtr req,
                                        [[maybe_unused]] SetLineSensor::Response::SharedPtr res) {
  char msg[MSG::MBL];
  snprintf(msg, MSG::MBL, LineSensorProxy::SENSOR_CONFIG, req->on, req->white, req->high_power,
           req->tilt, req->cross_th, req->wide, req->swap);
  sendToTeensy(MSG::make(msg), false);
  _internal_req = req;
}

void LineSensorProxy::setSensorOnOff(const ToggleLineSensor::Request::SharedPtr req,
                                     [[maybe_unused]] ToggleLineSensor::Response::SharedPtr res) {
  switch (req->mode) {
    case ToggleLineSensor::Request::TOGGLE:
      _internal_req->on = !_internal_req->on;
      break;
    case ToggleLineSensor::Request::OFF:
      _internal_req->on = false;
      break;
    default:
      _internal_req->on = true;
      break;
  }
  setLineSensorMode(_internal_req);
}

void LineSensorProxy::decode(char* msg) {
  // like: liv <> <> <> <> <> <> <> <>
  /* liv + 8 values (one for each sensor)
   */
  if (strlen(msg) <= 4) return;
  const char* p1 = msg + 4;
  _msg.stamp = clock->now();
  _msg.l1 = strtoll(p1, (char**)&p1, 10);
  _msg.l2 = strtoll(p1, (char**)&p1, 10);
  _msg.l3 = strtoll(p1, (char**)&p1, 10);
  _msg.l4 = strtoll(p1, (char**)&p1, 10);
  _msg.l5 = strtoll(p1, (char**)&p1, 10);
  _msg.l6 = strtoll(p1, (char**)&p1, 10);
  _msg.l7 = strtoll(p1, (char**)&p1, 10);
  _msg.l8 = strtoll(p1, (char**)&p1, 10);

  publisher->publish(_msg);
}

}  // namespace raubase::teensy::proxy