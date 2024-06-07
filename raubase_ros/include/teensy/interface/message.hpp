#ifndef RAUBASE_TENSY_MESSAGE
#define RAUBASE_TENSY_MESSAGE

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

#include <cstring>
#include <memory>

#include "common/utils/types.hpp"
#include "common/utils/utime.hpp"

namespace raubase::teensy {

struct MSG {
  // =================================================================
  //                        Static methods
  // =================================================================
  static constexpr int MML = 400;                //< Max length for USB Messaging
  static constexpr int MCL = 3;                  //< Max length for CRC validation
  static constexpr int MPL = MCL + 1;            //< Max length for prefix (CRC+confirmation)
  static constexpr int MBL = MML - MPL;          //< Max Length for body slot
  static constexpr const char SOL{';'};          //< Command starting character
  static constexpr const char EOL{'\n'};         //< End of line character for USB Messaging
  static constexpr const char EOS{'\0'};         //< End of string character
  static constexpr const char REQ_CONFIRM{'!'};  //< Character for requesting a confirmation

  /**
   * @brief Generate a 3 character Cyclic Redundancy Check (in the format ';NN') and put it in the
   * first 3 characters. NN is the (sum of character values modulus 99) + 1.
   * Only characters with a values > ' ' counts.
   *
   * @param msg the message to compute the CRC of, and where to put the result in.
   */
  static void generateCRC(char* msg);

  /**
   * Check for CRC error
   * \param rawMsg is the message preceded by crc
   * \return true if OK
   */
  static bool checkCRC(const char* msg);

  /**
   * @brief Make a new message object. Return a shared pointer to the message.
   *
   * @param msg the message to send
   * @param confirm whether this message need a confirmation
   * @return sptr<MSG> a shared point to the message.
   */
  static sptr<MSG> make(const char* msg, bool confirm = true) {
    return std::make_shared<MSG>(msg, confirm);
  }

  // =================================================================
  //                          Message Struct
  // =================================================================
  char msg[MML];             //< Message to send
  char _debug_msg[MML - 1];  //< Message to send (DEBUG, no end of line)
  int len;                   //< Length of the message to send
  bool need_confirm;         //< Need a confirmation from the Teensy Board
  bool ok = false;           //< If the message was correctly computed
  bool sent = false;         //< If the message was sent
  short sent_count = 0;      //< The number of time the message was sent
  UTime time_sent;           //< The time at which the messange was last sent
  MSG(const char* msg, bool confirm = true);

  /**
   * @brief Compare this message with its confirmation
   *
   * @return true
   * @return false
   */
  bool compare(const char* confirm) const {  // ignore potential \n
    return std::strncmp(&msg[MSG::MCL], confirm, std::strlen(confirm) - 1) == 0;
  }
};

}  // namespace raubase::teensy

#endif