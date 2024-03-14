#ifndef RAUBASE_LINE_SENSOR_PROXY
#define RAUBASE_LINE_SENSOR_PROXY

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

#include "raubase_msgs/msg/data_line_sensor.hpp"
#include "raubase_msgs/msg/set_line_sensor_config.hpp"
#include "raubase_msgs/srv/toggle_line_sensor.hpp"
#include "teensy/interface/proxy_interface.hpp"

using raubase_msgs::msg::DataLineSensor;
using raubase_msgs::msg::SetLineSensorConfig;
using raubase_msgs::srv::ToggleLineSensor;

namespace raubase::teensy::proxy {

/**
 * @brief Proxy for converting Teensy Encoder messages into a ROS data message.
 */
class LineSensorProxy : public TeensyProxy {
  // =================================================================
  //                             Constants
  // =================================================================
 protected:
  static constexpr const char* TEENSY_COMP{"liv"};  //< Teensy component to subscribe from
  static constexpr const char* SENSOR_CONFIG{
      "lip %d %d %d %d %d %d %d"};  //< Message for configuring the sensor
  static constexpr const char* PUBLISHING_TOPIC{"line_sensor"};  //< Encoder state topic
  static constexpr const char* SETTING_SERVICE{
      "line_sensor/set"};  //< Service name for setting the sensors mode
  static constexpr const char* TOGGLE_SERVICE{
      "line_sensor/toggle"};  //< Service name for toggling the sensor
  static constexpr int QOS{10};

 public:
  static constexpr const char* NODE_NAME{"LineSensorProxy"};  //< ROS Node name
  static constexpr const char* TEENSY_MSG = TEENSY_COMP;  //< Teensy board receiving prefix //< QOS
                                                          // for all components

  // =================================================================
  //                             Proxy Methods
  // =================================================================
 public:
  LineSensorProxy(SendingCallback _clbk) : TeensyProxy(_clbk, NODE_NAME) {}
  void setupParams(rclcpp::Node::SharedPtr) override;
  void setupSubscriptions() override;
  void decode(char*) override;
  void closeTeensy() override;

  // =================================================================
  //                          Component Methods
  // =================================================================
 protected:
  /**
   * @brief Set the line sensors with the given options (on/off, high/low power ...).
   */
  void setLineSensorMode(const SetLineSensorConfig::SharedPtr);

  /**
   * @brief Turn the sensor on / off
   */
  void setSensorOnOff(const ToggleLineSensor::Request::SharedPtr,
                      ToggleLineSensor::Response::SharedPtr);

  // =================================================================
  //                             ROS Members
  // =================================================================
 protected:
  DataLineSensor _msg;                                     //< Message to send
  rclcpp::Publisher<DataLineSensor>::SharedPtr publisher;  //< Data publisher
  rclcpp::Subscription<SetLineSensorConfig>::SharedPtr
      setting_sub;  //< Service for setting the sensor params
  rclcpp::Service<ToggleLineSensor>::SharedPtr toggling_srv;  //< Servive for on/off
  rclcpp::Clock::SharedPtr clock;  //< ROS clock for stamping the messages

  // =================================================================
  //                           Sensor Parameters
  // =================================================================
  bool _on;                                      //< Whether the sensor is on
  int _refresh_rate;                             //< The refresh rate for the line sensors
  SetLineSensorConfig::SharedPtr _internal_req;  //< Internal request for setting modes
};

}  // namespace raubase::teensy::proxy

#endif