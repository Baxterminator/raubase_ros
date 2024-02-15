#ifndef RAUBASE_TENSY_MESSAGE
#define RAUBASE_TENSY_MESSAGE

#include <cstring>

namespace raubase::teensy {

struct MSG {
  // =================================================================
  //                        Static methods
  // =================================================================
  static constexpr int MML = 400;                //< Max length for USB Messaging
  static constexpr int MCL = 3;                  //< Max length for CRC validation
  static constexpr int MPL = MCL + 1;            //< Max length for prefix (CRC+confirmation)
  static constexpr int MSL = MML - MCL;          //< Max Length for body slot
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

  // =================================================================
  //                          Message Struct
  // =================================================================
  char msg[MML];
  int len;
  bool ok = false;
  short sent_count = 0;
  MSG(const char* msg);

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