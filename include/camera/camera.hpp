#ifndef RAUBASE_CAMERA
#define RAUBASE_CAMERA

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

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <robotbot_msgs/srv/ask_camera_image.hpp>
#include <robotbot_msgs/srv/detail/ask_camera_image__struct.hpp>
#include <robotbot_msgs/srv/set_camera_mode.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace raubase::cam {

using robotbot_msgs::srv::AskCameraImage;
using robotbot_msgs::srv::SetCameraMode;
using sensor_msgs::msg::Image;
using std::chrono::milliseconds;
using namespace sensor_msgs::image_encodings;

using namespace std::chrono_literals;

/**
 * @brief Camera interface for the robobot.
 */
class Camera : public rclcpp::Node {
  // ==========================================================================
  //                                 Methods
  // ==========================================================================
 public:
  // ----------------------------- Life Cycle ---------------------------------
  Camera(rclcpp::NodeOptions);

  /**
   * @brief Try to connect to the camera.
   * Initialize all settings for the camera
   */
  void try_connect();

  /**
   * @brief Close every handle before returning
   */
  void terminate();

  /**
   * @brief Camera node destructor
   */
  ~Camera();

  // ------------------------- Running methods --------------------------------

 private:
  /**
   * @brief Run the image extractor.
   */
  void run();

  /**
   * @brief Grab the last image and put it inside the image message
   */
  Image::SharedPtr grabLastImage();

  void set_on_demand(SetCameraMode::Request::ConstSharedPtr req,
                     [[maybe_unused]] SetCameraMode::Response::SharedPtr res) {
    on_demand = req->on_demand;
  }

  void get_image([[maybe_unused]] AskCameraImage::Request::ConstSharedPtr req,
                 AskCameraImage::Response::SharedPtr res) {
    auto img = grabLastImage();
    res->image = *img;
    _img_pub->publish(*img);
  }

  // ==========================================================================
  //                                    Members
  // ==========================================================================
 private:
  // --------------------------- Data Connection ------------------------------
  static constexpr int CAM_API{cv::CAP_V4L2};
  long _checking_s;
  std::string device;
  rclcpp::TimerBase::SharedPtr checker;

  // ------------------------------ Camera Prop -------------------------------
  static constexpr const char* VIDEO_FORMAT{"MJPG"};  //< The format for the input video
  unsigned long CC4;                                  //< CC4 instance
  int img_width;                                      //< Width of the image to read
  int img_height;                                     //< Height of the image to read
  double img_fps;                                     //< FPS to the video to read
  cv::VideoCapture _cam;                              //< Camera handle
  bool on_demand;  //< Whether the pictures should be sent on  demand or at request

  // ----------------------------- Camera Reading -----------------------------
  static constexpr const char* IMG_ENCODING{BGR8};
  static constexpr int QOS{10};
  cv::Mat frame;
  cv_bridge::CvImage msg;
  rclcpp::TimerBase::SharedPtr runner;
  rclcpp::Publisher<Image>::SharedPtr _img_pub;

  // ----------------------------- Node service -----------------------------
  static constexpr const char* NODE_NAME{"camera"};

  static constexpr const char* MODE_SET_SRV{"/camera/set_mode"};
  rclcpp::Service<SetCameraMode>::SharedPtr srv_mode_set;

  static constexpr const char* ASK_IMG_SRV{"/camera/get_image"};
  rclcpp::Service<AskCameraImage>::SharedPtr srv_ask_img;
};

}  // namespace raubase::cam

#endif