#include <asm-generic/errno-base.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "common/utime.hpp"
#include "teensy/teensy.hpp"
#include "teensy/usb_connection.hpp"

namespace raubase::teensy {

void Teensy::send(sptr<MSG> msg, bool direct) {
  if (direct) sendDirect(msg);
  TX_queue.push(msg);
}

void Teensy::receiveUSB(char* msg) {
  // If receiving a comment message from the Teensy
  if (std::strncmp(msg, "#", 1) == 0) {
    RCLCPP_INFO(get_logger(), "%s", msg);
    return;
  }

  // Else if message to process, send it to the right proxy
  RCLCPP_INFO(get_logger(), "Receiving msg %s", msg);
  for (auto& [sub_prefix, proxy] : _proxies_mapping) {
    if (std::strncmp(sub_prefix, msg, std::strlen(sub_prefix)) == 0) {
      _proxies[proxy]->decode(msg);
      return;
    }
  }
  RCLCPP_WARN(get_logger(), "Received message uncaught by proxies! (%s)", msg);
}

///////////////////////////////////////////////////////////////////////////////

void Teensy::confirmMessage(const char* msg) {
  // Test if messages in queue
  if (TX_queue.isEmpty()) return;

  // Get queue lock
  TX_queue.lock();
  auto front_obj = TX_queue.front();

  // Test if first message in queue has already been sent
  if (front_obj->sent_count != 0 && front_obj->compare(&msg[Teensy::CONFIRM_LENGTH + MSG::MPL])) {
    RCLCPP_INFO(get_logger(), "Confirmation receivedfor message (%s)", front_obj->_debug_msg);
    front_obj = nullptr;
    TX_queue.unlock();
    TX_queue.pop();
  }
  TX_queue.unlock();
}

///////////////////////////////////////////////////////////////////////////////

void Teensy::fetchRX() {
  int n = read(usb_co.port, &usb_co.RX_buffer[usb_co.rxIdx], 1);

  // If error and not asked a EAGAIN
  if (n < 0 && errno != EAGAIN) {
    RCLCPP_ERROR(get_logger(), "Teensy port error");
    usleep(100000);  // Sleep for 100ms
    closeUSB();
    return;
  }
  // Had too much data when only requested 1 byte
  else if (n > 1) {
    RCLCPP_ERROR(get_logger(), "Teensy RX got %d chars when only asking for 1", n);
    fflush(nullptr);
    return;
  }
  // If no byte read, nothing to process
  else if (n == 0)
    return;

  // Detect cmd beginning byte
  // rxIdx = 0 until a command is found: (char stream starting with a ';')
  // When a command is found, increment rxIdx to fill the next cell
  if (usb_co.rxIdx == 0 && usb_co.RX_buffer[0] == MSG::SOL) {
    usb_co.rxIdx = 1;
    return;
  } else if (usb_co.rxIdx > 0)
    usb_co.rxIdx++;

  // Detect cmd end and set the command end (with the '\0' character)
  if (usb_co.RX_buffer[usb_co.rxIdx - 1] != MSG::EOL) return;
  usb_co.RX_buffer[usb_co.rxIdx] = MSG::EOS;

  // Check CRC encoding
  if (!MSG::checkCRC(usb_co.RX_buffer))
    RCLCPP_DEBUG(get_logger(), "CRC check gave false for message \"%s\"", usb_co.RX_buffer);
  // Check if confirmation message
  else if (strncmp(&usb_co.RX_buffer[MSG::MCL], CONFIRM_MSG, CONFIRM_LENGTH) == 0)
    confirmMessage(usb_co.RX_buffer);
  // Else process it like a sensor signal (remove the CRC)
  else
    receiveUSB(&usb_co.RX_buffer[MSG::MCL]);

  // Reset RX state after processing the command
  usb_co.connecting = false;
  usb_co.hasRecentActivity = true;
  usb_co.lastRXTime.now();
  usb_co.rxIdx = 0;
  n = 0;
}

void Teensy::sendTX() {
  if (TX_queue.isEmpty()) return;

  auto obj = TX_queue.front();

  // If confirmation has timed out, resend it or drop it
  if (obj->sent && obj->time_sent.getTimePassed() > _confirm_timeout) {
    RCLCPP_WARN(get_logger(), "Confirmation timed out for message (%s)", obj->_debug_msg);
    if (obj->sent_count >= _max_resend_cnt) {
      RCLCPP_ERROR(get_logger(), "TX Message runned out of confirmation and has been dropped");
      TX_queue.pop();
    } else
      obj->sent = false;
  }

  // If first message non sent, send it
  if (!obj->sent) {
    sendDirect(obj);

    if (!obj->need_confirm) TX_queue.pop();
  }
}

void Teensy::sendDirect(sptr<MSG> msg) {
  if (!usb_co.opened) return;

  RCLCPP_INFO(get_logger(), "Sending message (%s)", msg->_debug_msg);
  usb_co._tx_lock.lock();
  int n = msg->len;
  int sent = 0, last_sent = 0, iterations = 0;
  bool lostConnection = false;
  while ((sent < n) && (iterations < 100)) {
    last_sent = write(usb_co.port, &msg->msg[sent], msg->len - sent);

    // Check any error
    if (last_sent < 0) {
      switch (errno) {
        case EAGAIN:
          // Buffer full, waiting
          // not all send - just continue
          printf("STeensy::sendDirect: waiting - nothing send %d/%d\n", sent, n);
          usleep(_timer_wait.count());
          iterations += 1;
          break;
        default:
          RCLCPP_ERROR(get_logger(), "Closing connection on TX!");
          lostConnection = true;
          break;
      }
      // dump the rest on most errors
      if (lostConnection) break;
    } else {
      // Count bytes sent
      sent += last_sent;
    }
  }

  msg->sent_count++;
  msg->sent = true;
  msg->time_sent.now();
  usleep(_timer_wait.count());
  usb_co._tx_lock.unlock();
}

///////////////////////////////////////////////////////////////////////////////

void Teensy::TRXLoop() {
  static UTime loop_time("now");

  // Check for connections errors
  if (usb_co.connectionTimeout(_activity_timeout, _connect_timeout)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 200, "USB connection timed out !");
    closeUSB();
    return;
  } else if (!usb_co.opened) {
    openUSB();
    return;
  }

  // Activity check
  if (usb_co.hasRecentActivity && usb_co.lastRXTime.getTimePassed() > 2.0)
    usb_co.hasRecentActivity = false;

  // Communication
  if (usb_co.opened) fetchRX();
  if (usb_co.opened) sendTX();

  // Check for NTP updates
  usb_co.NTPUpdate = (loop_time.getTimePassed() > 2.0);
  if (usb_co.NTPUpdate)
    RCLCPP_INFO(get_logger(), "NTP Update ? Time glitch of %.3f sec", loop_time.getTimePassed());
  loop_time.now();
}

}  // namespace raubase::teensy