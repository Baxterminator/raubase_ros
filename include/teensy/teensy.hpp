#ifndef RAUBASE_TEENSY
#define RAUBASE_TEENSY

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

#include <chrono>
#include <cstring>
#include <map>
#include <queue>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

#include "common/shared_queue.hpp"
#include "teensy/interface/message.hpp"
#include "teensy/interface/proxy_interface.hpp"
#include "teensy/proxy/encoder.hpp"
#include "teensy/usb_connection.hpp"

namespace raubase::teensy {

class Teensy : public rclcpp::Node {
  // =================================================================
  //                             Methods
  // =================================================================
 public:
  // ----------------------------- Life Cycle ---------------------------------

  /**
   * @brief Construct a new ROS Node providing USB connection to the Teensy board.
   *   Also initialize basic ROS parameters.
   * @param parameter opts - Options for making the ROS Node
   */
  Teensy(rclcpp::NodeOptions);

  /**
   * @brief Setup all subcomponents needed for this node.
   */
  void setupProxy();

  /**
   * @brief Destructor of the Teensy Node to close all connections.
   */
  ~Teensy();

 private:
  // ------------------------------ USB Exchange ------------------------------

  /**
   * @brief Send the given message to Teensy board.
   * @param msg - the message to send
   * @param direct - whether to directly send it to the board or to place in the queue.
   */
  void send(sptr<MSG> msg, bool direct);

  /**
   * @brief Send the given message directly to the Teensy board without waiting in the queue.
   * @param msg the message to send
   */
  void sendDirect(sptr<MSG> msg);

  /**
   * @brief Decode and process a message from the Teensy board.
   */
  void receiveUSB(char*);

  /**
   * @brief Main Loop for reading and writing streams on the USB serial port.
   */
  void TRXLoop();

  /**
   * @brief Fetch the next byte in the RX stream.
   * Assemble the whole command with one byte per loop.
   * Launch decode sequence when getting an end-of-line char.
   */
  void fetchRX();

  /**
   * @brief Send the next message in the queue if possible (no msg waiting for a confirmation ).
   */
  void sendTX();

  /**
   * @brief Confirm that the first message in the queue got a response from the teensy and pop it.
   */
  void confirmMessage(const char*);

  // ------------------------------- USB Setup --------------------------------

  /**
   * @brief Open the USB connection.
   */
  void openUSB();

  /**
   * @brief Close the USB connection.
   */
  void closeUSB();

  // -------------------------------- Custom ----------------------------------

  // =================================================================
  //                             Members
  // =================================================================
 public:
  static constexpr const char* NODE_NAME{"teensy"};     //< ROS Node name
  static constexpr int REGBOT_HW{-1};                   //< RegBot hardware model
  static constexpr const char* CONFIRM_MSG{"confirm"};  //< Confirmation message
  static constexpr int CONFIRM_LENGTH = 7;  //< Confirmation message length (length of CONFIRM_MSG)

 private:
  // ------------------------------- ROS Object -------------------------------

  rclcpp::TimerBase::SharedPtr trx_runtime = nullptr;
  std::chrono::microseconds _timer_period;
  std::chrono::microseconds _timer_wait;

  // ------------------------------ Configurations ----------------------------

  std::string _device;      //< Device name (e.g. robotbot)
  float _confirm_timeout;   //< Time for confirming the timeout
  float _connect_timeout;   //< Timeout when connecting to the device
  float _activity_timeout;  //< Timeout for activity, if nothing happens, stop the connection
  USBConnection usb_co;     //< USB Connection data

  // ------------------------- Message encoding / decoding --------------------
  const SendingCallback _sending_cbk = [this](sptr<MSG> msg, bool direct) {
    this->send(msg, direct);
  };
  std::map<const char*, TeensyProxy::SharedPtr> converters = {
      {"enc", TeensyProxy::make_shared<proxy::EncoderProxy>(_sending_cbk)},
  };                                //< Converter map
  SharedQueue<sptr<MSG>> TX_queue;  //< Queue for sending cmd to the teensy board
};

}  // namespace raubase::teensy

#endif