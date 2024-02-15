#include "teensy/interface/message.hpp"

#include <cctype>
#include <cstdio>
#include <cstring>
#include <rclcpp/logging.hpp>

namespace raubase::teensy {

MSG::MSG(const char *message) {
  // Verifying Message Length
  len = strnlen(message, MML);
  ok = (len + 5) < MML;
  if (!ok) return;

  // Make actual message
  len += MCL;
  msg[MPL] = REQ_CONFIRM;
  strncpy(&msg[4], message, len);

  // EOL + End of string verification
  if (msg[len - 1] != EOL) msg[len++] = EOL;
  msg[len] = '\0';

  MSG::generateCRC(msg);
}

void MSG::generateCRC(char *msg) {
  int n = strlen(msg);
  const char *p1 = &msg[3];
  int sum = 0;
  for (int i = 0; i < n; i++) {  // do not count \t, \r, \n etc
    // as these gives problems for systems with auto \n or \n\r or similar
    if (*p1 >= ' ') sum += *p1;
    if (*p1 == '\n') {  // one newline is allowed only, the rest is ignored
      break;
    }
    p1++;
  }
  snprintf(msg, MCL, ";%02d", (sum % 99) + 1);
}

bool MSG::checkCRC(const char *msg) {
  // Check if CRC check code
  if (msg[0] != MSG::SOL) return false;
  // Check if CRC is here
  if (!std::isdigit(msg[1]) || !std::isdigit(msg[2])) return false;

  const char *p1 = &msg[MSG::MCL];
  int sum = 0;
  int m = strlen(p1);
  for (int i = 0; i < m; i++) {  // sum all visible characters
    if (*p1 >= ' ') sum += *p1;
    p1++;
  }
  int q1 = (sum % 99) + 1;                      // Computed CRC
  int q2 = (msg[1] - '0') * 10 + msg[2] - '0';  // Transmitted CRC

  return q1 == q2;
  // if (q1 != q2)
  //   printf("# UHandler::handleCommand: CRC check failed (from Teensy) q1=%d != q2=%d (msg=%s\n",
  //   q1,
  //          q2, msg);
  // return true;
}

}  // namespace raubase::teensy