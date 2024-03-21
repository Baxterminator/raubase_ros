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
#include <memory>
#include <opencv2/videoio.hpp>
#include <raubase_msgs/action/ask_calibration.hpp>
#include <raubase_msgs/action/calib_camera.hpp>
#include <raubase_msgs/msg/set_camera_mode.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/empty.hpp>

#include "common/utils/types.hpp"

namespace raubase::cam {

// Action definitions

namespace ra = rclcpp_action;
using raubase_msgs::action::AskCalibration;
using raubase_msgs::action::CalibCamera;
using SrvGoalHandle = ra::ServerGoalHandle<AskCalibration>;
using CltGoalHandle = ra::ClientGoalHandle<CalibCamera>;
using CalibClient = ra::Client<CalibCamera>;
using CalibServer = ra::Server<AskCalibration>;

// Messages
using raubase_msgs::msg::SetCameraMode;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;
using namespace sensor_msgs::image_encodings;
using std_msgs::msg::Empty;

// utils
using std::chrono::milliseconds;
using namespace std::chrono_literals;

/**
 * @brief Camera interface for the robobot.
 */
class Camera : public rclcpp::Node {
  // ==========================================================================
  //                                 Constants
  // ==========================================================================
 private:
 public:
  // ----------------------------- ROS Operations -----------------------------
  static constexpr const char* NODE_NAME{"camera"};
  static constexpr int QOS{10};
  static constexpr int THROTTLED_MS{500};

  struct Params {
    // Internal functionnement
    static constexpr const char* CO_TIMEOUT{"co_s"};
    static constexpr const char* ON_TRIGGER{"on_trigger"};

    // Camera Properties
    static constexpr const char* DEVICE{"device"};
    static constexpr const char* WIDTH{"width"};
    static constexpr const char* HEIGHT{"height"};

    // Calibration
    static constexpr const char* ONLINE_TIMEOUT{"online_timeout_ms"};
    static constexpr const char* CALIB_WIDTH{"calib_width"};
    static constexpr const char* CALIB_HEIGHT{"calib_height"};
    static constexpr const char* K{"calib_K"};
    static constexpr const char* D{"calib_d"};
  };

  struct Default {
    // Internal functionnement
    static constexpr double CO_TIMEOUT_S{1.0};
    static constexpr const char* DEVICE_PREFIX{"/dev/video"};
    static constexpr bool ON_TRIGGER{true};

    // Camera Properties
    static constexpr int DEVICE{0};
    static constexpr int WIDTH{-1};
    static constexpr int HEIGHT{-1};

    // Calibration
    static constexpr int ONLINE_TIMEOUT{500};
    static constexpr int CALIB_WIDTH{1080};
    static constexpr int CALIB_HEIGHT{720};
    static constexpr double CALIB_Fx{1.0};
    static constexpr double CALIB_Fy{1.0};
    static constexpr int CALIB_Cx{CALIB_WIDTH / 2};
    static constexpr int CALIB_Cy{CALIB_HEIGHT / 2};
    static constexpr double CALIB_D{0.0};
  };

  struct Topics {
    static constexpr const char* OUT_RAW{"camera/raw"};
    static constexpr const char* OUT_COMPRESSED{"camera/compressed"};
    static constexpr const char* IN_TRIGGER{"camera/trigger"};
    static constexpr const char* IN_SET_MODE{"camera/set_mode"};
    static constexpr const char* OUT_CAM_INFO{"camera/cam_info"};
  };

  struct Actions {
    static constexpr const char* IN_LAUNCH_CALIB{"calibrate"};
    static constexpr const char* OUT_LAUNCH_CALIB{"/calib/camera"};
  };

  // -------------------------- Camera Connection -----------------------------
  static constexpr int CAM_API{cv::CAP_V4L2};
  static constexpr const char* VIDEO_FORMAT{"MJPG"};  //< The format for the input video
  static constexpr const char* IMG_ENCODING{BGR8};    //< Image encoding
  static constexpr int BUFFER_SIZE{1};                //< Buffer size for camera

  // ==========================================================================
  //                                 Methods
  // ==========================================================================
 public:
  /////////////////////////////////////////////////////////////////////////////
  // ----------------------------- Life Cycle ---------------------------------
  /////////////////////////////////////////////////////////////////////////////
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

  /////////////////////////////////////////////////////////////////////////////
  // --------------------- Camera Calibration methods -------------------------
  /////////////////////////////////////////////////////////////////////////////
 private:
  /**
   * @brief Set the up actions for the calibration purpose.
   */
  void setup_actions();

  /**
   * @brief Initialize the camera info object from the parameters
   */
  void init_cam_info();

  /**
   * @brief Share the camera info on the DDS network.
   */
  void share_cam_info();

  // Server

  /**
   * @brief Handle a request for calibration.
   */
  ra::GoalResponse srv_handle_goal(const ra::GoalUUID&, AskCalibration::Goal::ConstSharedPtr);

  /**
   * @brief Handle if the calibration request has been cancelled by client.
   */
  ra::CancelResponse srv_handle_cancel(const sptr<SrvGoalHandle>);

  /**
   * @brief Launch the calibration request
   */
  void srv_handle_accepted(const sptr<SrvGoalHandle>);

  // Client

  /**
   * @brief Handle the response from the calibration server (accepted or rejected).
   */
  void accepted_callback(CltGoalHandle::ConstSharedPtr);

  /**
   * @brief Handle the calibration server feedback.
   */
  void feedback_callback(CltGoalHandle::SharedPtr, const CalibCamera::Feedback::ConstSharedPtr);

  /**
   * @brief Handle the result from the calibration server.
   */
  void result_callback(const CltGoalHandle::WrappedResult&);

  /////////////////////////////////////////////////////////////////////////////
  // --------------------------- Running methods------------------------------
  /////////////////////////////////////////////////////////////////////////////
 private:
  /**
   * @brief Run the image extractor on continuous mode.
   */
  void run();

  /**
   * @brief Grab the last image and put it inside the image message
   */
  void grabLastImage();

  /**
   * @brief Change the camera functionnement (on trigger or continous)
   */
  void set_on_demand(SetCameraMode::ConstSharedPtr msg) { on_trigger = msg->on_demand; }

  /**
   * @brief Trigger the camera capture and send it to the channels
   *
   */
  void get_image(Empty::SharedPtr) {
    // Check if camera has been opened
    if (!_cam.isOpened()) return;

    grabLastImage();
    _img_pub->publish(*msg.toImageMsg());
    _compr_pub->publish(*msg.toCompressedImageMsg());
    cam_info_pub->publish(cam_info);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), THROTTLED_MS, "Sending image ...");
  }

  // ==========================================================================
  //                                 Members
  // ==========================================================================
  /////////////////////////////////////////////////////////////////////////////
  // --------------------------- Data Connection ------------------------------
  /////////////////////////////////////////////////////////////////////////////
 private:
  milliseconds _co_timeout;
  std::string device;
  rclcpp::TimerBase::SharedPtr checker;
  rclcpp::TimerBase::SharedPtr runner;

  /////////////////////////////////////////////////////////////////////////////
  // ---------------------------- Mode functions ------------------------------
  /////////////////////////////////////////////////////////////////////////////
  rclcpp::Subscription<SetCameraMode>::SharedPtr sub_mode_set;
  rclcpp::Subscription<Empty>::SharedPtr sub_ask_img;

  /////////////////////////////////////////////////////////////////////////////
  // ------------------------------ Calibration -------------------------------
  /////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<CameraInfo>::SharedPtr cam_info_pub;

  milliseconds action_online_timeout;
  CalibClient::SharedPtr calib_client;
  CalibServer::SharedPtr calib_server;
  CltGoalHandle::ConstSharedPtr clt_handle = nullptr;
  sptr<SrvGoalHandle> srv_handle = nullptr;
  bool in_calibration = false;

  const sptr<AskCalibration::Feedback> srv_feedback = std::make_shared<AskCalibration::Feedback>();
  const sptr<AskCalibration::Result> result = std::make_shared<AskCalibration::Result>();

  /////////////////////////////////////////////////////////////////////////////
  // ------------------------------ Camera Prop -------------------------------
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  unsigned long CC4;      //< CC4 instance
  int img_width;          //< Width of the image to read
  int img_height;         //< Height of the image to read
  bool on_trigger;        //< Whether the pictures should be sent on  demand or at request
  CameraInfo cam_info;    //< Camera info object to send accross the whole project
  cv::VideoCapture _cam;  //< Camera handle

  /////////////////////////////////////////////////////////////////////////////
  // ----------------------------- Camera Process -----------------------------
  /////////////////////////////////////////////////////////////////////////////
  cv::Mat frame;
  cv_bridge::CvImage msg;
  rclcpp::Publisher<Image>::SharedPtr _img_pub;
  rclcpp::Publisher<CompressedImage>::SharedPtr _compr_pub;
};

}  // namespace raubase::cam

#endif