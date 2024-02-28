#include "teensy/proxy/distance.hpp"

#include <cstdio>

#include "teensy/interface/proxy_interface.hpp"

namespace raubase::teensy::proxy {

char strToIRType(const char* msg) {
  if (strcmp(msg, "sharp") == 0)
    return DistanceData::SHARP;
  else if (strcmp(msg, "URM09") == 0)
    return DistanceData::URM09;
  return DistanceData::UNKNOWN;
}

void DistanceProxy::setupParams(rclcpp::Node::SharedPtr node) {
  RCLCPP_INFO(logger, "Initializing proxy %s", NODE_NAME);

  // Declaring parameters for the encoder
  _refresh_rate = node->declare_parameter("ir_ms", 80);
  _sensor1_msg.calib = {node->declare_parameter("ir1_min_cm", 70000.),
                        node->declare_parameter("ir1_max_cm", 20000.)};
  _sensor1_msg.type = strToIRType(node->declare_parameter("ir1_type", "sharp").c_str());
  _sensor2_msg.calib = {node->declare_parameter("ir2_min_cm", 70000.),
                        node->declare_parameter("ir2_max_cm", 20000.)};
  _sensor2_msg.type = strToIRType(node->declare_parameter("ir2_type", "sharp").c_str());

  _urm_factor = node->declare_parameter("us_calib", 0.00126953125);  // 5.20m / 4096 (m per LSB)

  // Initializing working components
  sensor1_pub = node->create_publisher<DistanceData>(SENSOR_1_PUB_TOPIC, QOS);
  sensor2_pub = node->create_publisher<DistanceData>(SENSOR2_PUB_TOPIC, QOS);
  calib_srv = node->create_service<CalibrateDistanceSensor>(
      CALIBRATE_SRV, std::bind(&DistanceProxy::calibrateSensors, this, std::placeholders::_1,
                               std::placeholders::_2));
  clock = node->get_clock();
}

void DistanceProxy::setupSubscriptions() {
  RCLCPP_INFO(logger, "Initializing proxy %s", NODE_NAME);
  // Send calibration data
  auto req = std::make_shared<CalibrateDistanceSensor::Request>();
  req->sensor1 = _sensor1_msg.calib;
  req->sensor2 = _sensor2_msg.calib;
  calibrateSensors(req);

  // Subscribe to component
  subscribeTeensyComponent(TEENSY_COMP, _refresh_rate);
}

void DistanceProxy::calibrateSensors(
    const CalibrateDistanceSensor::Request::SharedPtr req,
    [[maybe_unused]] CalibrateDistanceSensor::Response::SharedPtr res) {
  RCLCPP_INFO(logger, "Sending calibration data to Teensy!");
  // Send to Teensy
  char cmd[MSG::MBL];
  std::snprintf(cmd, MSG::MBL, CALIB_CMD, req->sensor1[0], req->sensor1[1], req->sensor2[0],
                req->sensor2[1]);
  sendToTeensy(MSG::make(cmd), false);

  // Update inner data
  _sensor1_msg.calib[0] = req->sensor1[0];
  _sensor1_msg.calib[1] = req->sensor1[1];
  _sensor2_msg.calib[0] = req->sensor2[0];
  _sensor2_msg.calib[1] = req->sensor2[1];
}

void DistanceProxy::decode(char* msg) {
  // like: ir <ir13> <ir50> <ir13> <ir50>
  if (strlen(msg) < 3) return;
  const char* p1 = msg + 3;

  // Sensor 1 data
  _sensor1_msg.stamp = clock->now();
  _sensor1_msg.range = strtod(p1, (char**)&p1);
  _sensor1_msg.range_ad = strtod(p1, (char**)&p1);
  // If URM09, compute the range value
  if (_sensor1_msg.type == DistanceData::URM09)
    _sensor1_msg.range = _sensor1_msg.range_ad * _urm_factor;
  sensor1_pub->publish(_sensor1_msg);

  // Sensor 2 data
  _sensor2_msg.stamp = clock->now();
  _sensor2_msg.range = strtod(p1, (char**)&p1);
  _sensor2_msg.range_ad = strtod(p1, (char**)&p1);
  if (_sensor2_msg.type == DistanceData::URM09)
    _sensor2_msg.range = _sensor2_msg.range_ad * _urm_factor;
  sensor2_pub->publish(_sensor2_msg);
}

}  // namespace raubase::teensy::proxy