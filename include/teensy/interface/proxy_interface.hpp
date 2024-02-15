#ifndef RAUBASE_TEENSY_MSG_CONVERTER
#define RAUBASE_TEENSY_MSG_CONVERTER

/*
Copyright (C) 2017-2024 by DTU
jcan@dtu.dk
geoffrey.cote@centraliens-nantes.org

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

#include <cstdio>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include "common/types.hpp"
#include "teensy/interface/message.hpp"

namespace raubase::teensy {

typedef std::function<void(sptr<MSG>, bool)> SendingCallback;

/**
 * @brief Interface for for providing standardized message convertion between the ROS domain and the
 * teensy board.
 */
class TeensyProxy {
  // =================================================================
  //                             Proxy Work
  // =================================================================
 private:
  static constexpr const char *USING_INTERFACE_METHOD{"Trying to use interface method !"};

 public:
  typedef std::shared_ptr<TeensyProxy> SharedPtr;

  /**
   * @brief Make a shared ptr of the given Child Implementation.
   *
   * @tparam T the child class
   * @param _cbk the callback to send messages to the Teensy Board
   * @return SharedPtr
   */
  template <class T>
  static SharedPtr make_shared(SendingCallback _cbk) {
    return std::make_shared<T>(_cbk);
  }

  TeensyProxy(SendingCallback _cbk)
      : sendToTeensy(_cbk), logger(rclcpp::get_logger("TeensyProxy")) {}
  TeensyProxy(SendingCallback _cbk, const char *name)
      : sendToTeensy(_cbk), logger(rclcpp::get_logger(name)) {}

  // =================================================================
  //                             Proxy Methods
  // =================================================================

  /**
   * @brief Setup all needed components for making this proxy work (e.g. ROS publishers /
   * subscribers, ...).
   */
  virtual void setup(rclcpp::Node::SharedPtr) {
    RCLCPP_FATAL(logger, "%s", USING_INTERFACE_METHOD);
  };

  /**
   * @brief Decode a Teensy message and send it inside its own dedicated topic.
   *
   */
  virtual void decode(char *) { RCLCPP_FATAL(logger, "%s", USING_INTERFACE_METHOD); };

  // =================================================================
  //                           Utils Methods
  // =================================================================
 protected:
  /**
   * @brief Send a command to the Teensy board to subscribe to the given component
   */
  void subscribeTeensyComponent(const char *name, int period) {
    char cmd[MSG::MSL];
    snprintf(cmd, MSG::MSL, "sub %s %d", name, period);
    sendToTeensy(std::make_shared<MSG>(cmd), false);
  }

 protected:
  const SendingCallback sendToTeensy;  //< Callback to send the message to the Teensy board
  const rclcpp::Logger logger;         //< Logger for this proxy
};

}  // namespace raubase::teensy

#endif