#include "camera/camera.hpp"

#include <rclcpp/logging.hpp>
#include <vector>

#include "common/utils/rmw.hpp"
#include "common/utils/types.hpp"

namespace raubase::cam {

Camera::Camera(rclcpp::NodeOptions opts) : rclcpp::Node(NODE_NAME, opts) {
  RCLCPP_INFO(get_logger(), "Starting %s node!", NODE_NAME);
  RCLCPP_INFO(get_logger(), "Initializing parameters ...");
  // ---------------------------- Configurations ------------------------------
  _co_timeout =
      milliseconds((int)(declare_parameter(Params::CO_TIMEOUT, Default::CO_TIMEOUT_S) * 1000));
  device =
      Default::DEVICE_PREFIX + std::to_string(declare_parameter(Params::DEVICE, Default::DEVICE));

  img_width = declare_parameter(Params::WIDTH, Default::WIDTH);
  img_height = declare_parameter(Params::HEIGHT, Default::HEIGHT);
  on_trigger = declare_parameter(Params::ON_TRIGGER, Default::ON_TRIGGER);

  CC4 = cv::VideoWriter::fourcc(VIDEO_FORMAT[0], VIDEO_FORMAT[1], VIDEO_FORMAT[2], VIDEO_FORMAT[3]);

  // ------------------------------ Publisher ---------------------------------
  RCLCPP_INFO(get_logger(), "Initializing Publishers / Subscribers ...");
  msg.encoding = IMG_ENCODING;
  _img_pub = create_publisher<Image>(Topics::OUT_RAW, QOS);
  _compr_pub = create_publisher<CompressedImage>(Topics::OUT_COMPRESSED, QOS);
  cam_info_pub = create_publisher<CameraInfo>(Topics::OUT_CAM_INFO, TRANSIENT_QOS);

  // --------------------------------- Works ----------------------------------
  sub_mode_set = create_subscription<SetCameraMode>(
      Topics::IN_SET_MODE, QOS, std::bind(&Camera::set_on_demand, this, std::placeholders::_1));
  sub_ask_img = create_subscription<Empty>(
      Topics::IN_TRIGGER, QOS, std::bind(&Camera::get_image, this, std::placeholders::_1));

  // -------------------------- Camera connection -----------------------------
  RCLCPP_INFO(get_logger(), "Initializing Wall Timers ...");
  checker = create_wall_timer(_co_timeout, std::bind(&Camera::try_connect, this));
  runner = create_wall_timer(0ms, std::bind(&Camera::run, this));
  runner->cancel();

  // setup_actions();
  init_cam_info();
  RCLCPP_INFO(get_logger(), "The node has been successfully initialized!");
}

void Camera::terminate() {
  if (_cam.isOpened()) {
    _cam.release();
    RCLCPP_INFO(get_logger(), "Closed camera %s!", device.c_str());
  }
}

Camera::~Camera() { terminate(); }

///////////////////////////////////////////////////////////////////////////////

void Camera::try_connect() {
  if (_cam.isOpened()) {
    RCLCPP_INFO(get_logger(), "Cam %s already opened!", device.c_str());
    checker->cancel();
    runner->reset();
    return;
  }

  // Try to open camera
  _cam.open(device, CAM_API);
  if (!_cam.isOpened()) {
    RCLCPP_ERROR(get_logger(), "Couldn't open camera %s", device.c_str());
    return;
  }

  // Configure it
  RCLCPP_INFO(get_logger(), "Camera %s opened!", device.c_str());
  _cam.set(cv::CAP_PROP_FOURCC, CC4);
  if (img_width != -1)
    _cam.set(cv::CAP_PROP_FRAME_WIDTH, img_width);
  else
    img_width = _cam.get(cv::CAP_PROP_FRAME_WIDTH);
  if (img_height != -1)
    _cam.set(cv::CAP_PROP_FRAME_HEIGHT, img_height);
  else
    img_height = _cam.get(cv::CAP_PROP_FRAME_HEIGHT);

  _cam.set(cv::CAP_PROP_BUFFERSIZE, BUFFER_SIZE);

  RCLCPP_INFO(get_logger(), "Configuring camera to prop: %dx%d", img_width, img_height);
  checker->cancel();
  runner->reset();
}

///////////////////////////////////////////////////////////////////////////////

void Camera::grabLastImage() {
  _cam.read(msg.image);
  msg.header.stamp = get_clock()->now();
}

void Camera::run() {
  // Test if cam opened
  if (!_cam.isOpened()) {
    RCLCPP_ERROR(get_logger(), "Camera closed unexpectedly!");
    checker->reset();
    runner->cancel();
    return;
  }

  // Test if on demand
  if (on_trigger) return;

  // Else extract next image
  grabLastImage();
  _img_pub->publish(*msg.toImageMsg());
  _compr_pub->publish(*msg.toCompressedImageMsg());
  cam_info_pub->publish(cam_info);
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