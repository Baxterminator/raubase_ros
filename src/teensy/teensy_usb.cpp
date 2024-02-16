#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/logging.hpp>

#include "teensy/teensy.hpp"
#include "teensy/usb_connection.hpp"

namespace raubase::teensy {

void Teensy::closeUSB() {
  if (!usb_co.opened) return;

  RCLCPP_INFO(get_logger(), "Closing USB connection!");
  usleep(100000);  // Wait 100ms before closing

  // Close USB connection
  close(usb_co.port);
  usb_co.toDefault();
  RCLCPP_INFO(get_logger(), "USB connection closed!");

  // Flush the queue
  TX_queue.flush();
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Configurate the connection to be non-blocking.
 */
void configUSBConnection(USBConnection& usb) {
  int flags;
  if (-1 == (flags = fcntl(usb.port, F_GETFL, 0))) flags = 0;
  fcntl(usb.port, F_SETFL, flags | O_NONBLOCK);

  struct termios options;
  tcgetattr(usb.port, &options);
  options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;  //<Set baud rate
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcsetattr(usb.port, TCSANOW, &options);
  tcflush(usb.port, TCIFLUSH);
}

void Teensy::openUSB() {
  RCLCPP_INFO(get_logger(), "Trying to connect to USB device !");

  // If connected do nothing
  if (usb_co.port != -1) {
    RCLCPP_WARN(this->get_logger(), "Already connected to device %s !", usb_co.name.c_str());
    return;
  }

  // Else open the usb port
  usb_co.port = open(usb_co.name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  usb_co.opened = (usb_co.port != -1);
  if (!usb_co.opened) {
    RCLCPP_WARN(get_logger(), "Could not open connection with device %s", usb_co.name.c_str());
    usb_co.error++;
    usleep(300000);
  }

  // Configure everything
  configUSBConnection(usb_co);
  usb_co.error = 0;
  usb_co.connecting = true;
  RCLCPP_INFO(get_logger(), "Connection opened!");
  setupProxy();
}

}  // namespace raubase::teensy