#ifndef RAUBASE_DISTANCE_PROXY
#define RAUBASE_DISTANCE_PROXY

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

#include <rclcpp/service.hpp>
#include <raubase_msgs/srv/detail/calibrate_distance_sensor__struct.hpp>

#include "raubase_msgs/msg/distance_data.hpp"
#include "raubase_msgs/srv/calibrate_distance_sensor.hpp"
#include "teensy/interface/proxy_interface.hpp"

using raubase_msgs::msg::DistanceData;
using raubase_msgs::srv::CalibrateDistanceSensor;

namespace raubase::teensy::proxy {

/**
 * @brief Proxy for converting Teensy Encoder messages into a ROS data message.
 */
class DistanceProxy : public TeensyProxy {
  // =================================================================
  //                             Constants
  // =================================================================
 public:
  static constexpr const char* NODE_NAME{"DistanceProxy"};  //< ROS Node name
  static constexpr const char* TEENSY_COMP{"ir"};           //< Teensy component to subscribe from
  static constexpr const char* TEENSY_MSG = TEENSY_COMP;    //< Teensy board receiving prefix

 protected:
  static constexpr const char* SENSOR_1_PUB_TOPIC{"dist_1"};  //< Sensor1 data topic
  static constexpr const char* SENSOR2_PUB_TOPIC{"dist_2"};   //< Sensor2 data topic
  static constexpr const char* CALIB_CMD{
      "irc %f %f %f %f 1"};  //< Calibration message for the Teensy
  static constexpr const char* CALIBRATE_SRV{
      "distance/set_calib"};     //< Service to calibrate the sensors
  static constexpr int QOS{10};  //< QOS for all components

  // =================================================================
  //                             Proxy Methods
  // =================================================================
 public:
  DistanceProxy(SendingCallback _clbk) : TeensyProxy(_clbk, NODE_NAME) {}
  void setupParams(rclcpp::Node::SharedPtr) override;
  void setupSubscriptions() override;
  void decode(char*) override;

  // =================================================================
  //                          Component Methods
  // =================================================================
 protected:
  /**
   * @brief Set the calibration values to the sensors.
   */
  void calibrateSensors(const CalibrateDistanceSensor::Request::SharedPtr,
                        CalibrateDistanceSensor::Response::SharedPtr = nullptr);

 public:
  // =================================================================
  //                             ROS Members
  // =================================================================
 protected:
  DistanceData _sensor1_msg;
  DistanceData _sensor2_msg;
  rclcpp::Publisher<DistanceData>::SharedPtr sensor1_pub;
  rclcpp::Publisher<DistanceData>::SharedPtr sensor2_pub;
  rclcpp::Service<CalibrateDistanceSensor>::SharedPtr calib_srv;
  rclcpp::Clock::SharedPtr clock;  //< ROS clock for stamping the messages

  // =================================================================
  //                           Sensor Parameters
  // =================================================================
  int _refresh_rate;   //< The refresh rate for the encoders
  double _urm_factor;  //< Calibration value for URM09
};

}  // namespace raubase::teensy::proxy

#endif