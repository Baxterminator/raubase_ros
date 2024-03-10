#ifndef TEENSY_USB_CONNECTION
#define TEENSY_USB_CONNECTION

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

#include <mutex>
#include <string>

#include "common/utils/utime.hpp"
#include "teensy/interface/message.hpp"

namespace raubase::teensy {

/**
 * @brief Container for an USB connection data.
 */
struct USBConnection {
  // USB Device connection
  int port;          //< The serial port onto which the connection was opened
  bool opened;       //< If the connection was open successfully
  std::string name;  //< The name of the device

  // RX buffer
  char RX_buffer[MSG::MML];  //< The buffer in which to write the RX message
  short rxIdx = 0;           //< The index into which write the message

  // Time flags
  bool connecting;         //< The connection has just opened
  UTime connectTime;       //< The time at which the connection was opened
  bool hasRecentActivity;  //< Whether there was an RX activity recently
  UTime lastRXTime;        //< The last time a RX byte has been received

  // Error
  short error = 0;         //< The number of time the connection has failed to open successfully
  bool NTPUpdate = false;  //< Whether there was a NTP update during the loop

  // TX Lock
  std::mutex _tx_lock;

  USBConnection() { toDefault(); }

  void toDefault() {
    port = -1;
    opened = false;
    connecting = false;
    hasRecentActivity = false;
    lastRXTime.now();
    error = 0;
    rxIdx = 0;
    NTPUpdate = false;
  }

  /**
   * @brief Return whether the USB connection has timed out:
   *    - If connected, if no message has been received for a fixed duration -> timeout
   *    - If connecting, if the delay to connect has been passed -> timeout
   *
   * @param rx_timeout
   * @param connect_timeout
   * @return true
   * @return false
   */
  inline bool connectionTimeout(float rx_timeout, float connect_timeout) {
    return opened && ((!hasRecentActivity && lastRXTime.getTimePassed() > rx_timeout) ||
                      (connecting && connectTime.getTimePassed() > connect_timeout));
  }
};

}  // namespace raubase::teensy

#endif