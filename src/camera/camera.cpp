#include "camera/camera.hpp"

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <opencv2/videoio.hpp>

#include "common/types.hpp"

namespace raubase::cam {

Camera::Camera(rclcpp::NodeOptions opts) : rclcpp::Node("camera", opts) {
  // ---------------------------- Configurations ------------------------------
  _checking_s = declare_parameter("check_s", 1);
  device = declare_parameter("device", 0);

  img_width = declare_parameter("width", 1280);
  img_height = declare_parameter("height", 720);
  img_fps = declare_parameter("fps", 25);

  CC4 = cv::VideoWriter::fourcc(VIDEO_FORMAT[0], VIDEO_FORMAT[1], VIDEO_FORMAT[2], VIDEO_FORMAT[3]);

  // ------------------------------ Publisher ---------------------------------
  msg.encoding = IMG_ENCODING;
  _img_pub = create_publisher<Image>("camera", QOS);

  // -------------------------- Camera connection -----------------------------
  checker =
      create_wall_timer(milliseconds(_checking_s * 1000), std::bind(&Camera::try_connect, this));
  runner = create_wall_timer(milliseconds((int)std::floor(1 / img_fps * 1000)),
                             std::bind(&Camera::run, this));
  runner->cancel();
}

void Camera::terminate() { _cam.release(); }

Camera::~Camera() { terminate(); }

///////////////////////////////////////////////////////////////////////////////

void Camera::try_connect() {
  // Try to open camera
  _cam.open(device, CAM_API);
  if (!_cam.isOpened()) {
    RCLCPP_ERROR(get_logger(), "Couldn't open camera %zu", device);
    return;
  }

  // Configure it
  RCLCPP_INFO(get_logger(), "Camera %zu opened!", device);

  _cam.set(cv::CAP_PROP_FOURCC, CC4);
  _cam.set(cv::CAP_PROP_FRAME_WIDTH, img_width);
  _cam.set(cv::CAP_PROP_FRAME_HEIGHT, img_height);
  _cam.set(cv::CAP_PROP_FPS, img_fps);
  _cam.set(cv::CAP_PROP_CONVERT_RGB, true);

  RCLCPP_INFO(get_logger(), "Configuring camera to prop: %dx%d@%f", img_width, img_height, img_fps);
  checker->cancel();
  runner->reset();
}

///////////////////////////////////////////////////////////////////////////////

void Camera::run() {
  // Test if cam opened
  if (!_cam.isOpened()) {
    RCLCPP_ERROR(get_logger(), "Camera closed unexpectedly!");
    checker->reset();
    runner->cancel();
    return;
  }

  // Else extract next image
  _cam.read(frame);
  if (frame.empty()) return;
  msg.image = frame;
  msg.header.stamp = get_clock()->now();
  _img_pub->publish(*msg.toImageMsg());
}

}  // namespace raubase::cam

///////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<raubase::cam::Camera>(rclcpp::NodeOptions{});

  while (rclcpp::ok()) rclcpp::spin_some(node);
  node->terminate();

  rclcpp::shutdown();
}