#include <functional>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>

#include "camera/camera.hpp"
#include "common/utils/rmw.hpp"

namespace raubase::cam {

///////////////////////////////////////////////////////////////////////////////
// Calibration Initialization
///////////////////////////////////////////////////////////////////////////////

void Camera::init_cam_info() {
  RCLCPP_INFO(get_logger(), "Initializing camera information message ...");
  cam_info.header.frame_id = NODE_NAME;
  cam_info.header.stamp = get_clock()->now();

  // Image properties
  cam_info.width = declare_parameter(Params::CALIB_WIDTH, Default::CALIB_WIDTH);
  cam_info.height = declare_parameter(Params::CALIB_HEIGHT, Default::CALIB_HEIGHT);

  // Matrixes
  auto K = declare_parameter(Params::K, std::vector<double>{Default::CALIB_Fx, Default::CALIB_Fy,
                                                            Default::CALIB_Cx, Default::CALIB_Cy});
  cam_info.k[0] = K[0];
  cam_info.k[2] = K[2];
  cam_info.k[4] = K[1];
  cam_info.k[5] = K[3];

  auto d = declare_parameter(
      Params::D, std::vector<double>{Default::CALIB_D, Default::CALIB_D, Default::CALIB_D,
                                     Default::CALIB_D, Default::CALIB_D});
  cam_info.d.resize(d.size());
  for (int i = 0; i < 5; i++) cam_info.d[i] = d[i];
}

void Camera::setup_actions() {
  // Setup calibration publishing
  init_cam_info();

  RCLCPP_INFO(get_logger(), "Initializing CamInfo pub and share saved configuration ...");
  cam_info_pub = create_publisher<CameraInfo>(Topics::OUT_CAM_INFO, TRANSIENT_QOS);
  share_cam_info();

  // Setup actions
  RCLCPP_INFO(get_logger(), "Initializing calibration action server and client ...");
  action_online_timeout =
      milliseconds(declare_parameter(Params::ONLINE_TIMEOUT, Default::ONLINE_TIMEOUT));
  calib_client = ra::create_client<CalibCamera>(this, Actions::OUT_LAUNCH_CALIB);
  calib_server = ra::create_server<AskCalibration>(
      this, Actions::IN_LAUNCH_CALIB,
      std::bind(&Camera::srv_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Camera::srv_handle_cancel, this, std::placeholders::_1),
      std::bind(&Camera::srv_handle_accepted, this, std::placeholders::_1));
}

void Camera::share_cam_info() {
  RCLCPP_INFO(get_logger(), "Sending Camera Info on the network");
  cam_info_pub->publish(this->cam_info);
}

///////////////////////////////////////////////////////////////////////////////
// Action Server (Receiving from any client)
///////////////////////////////////////////////////////////////////////////////

ra::GoalResponse Camera::srv_handle_goal(const ra::GoalUUID&,
                                         AskCalibration::Goal::ConstSharedPtr) {
  if (in_calibration) {
    RCLCPP_WARN(get_logger(), "Asking to calibrate while already in calibration!");
    return ra::GoalResponse::REJECT;
  }
  RCLCPP_INFO(get_logger(), "Asking for calibration, launching it!");
  return ra::GoalResponse::ACCEPT_AND_EXECUTE;
}

ra::CancelResponse Camera::srv_handle_cancel(const sptr<SrvGoalHandle>) {
  RCLCPP_WARN(get_logger(), "Cancelling calibration!");
  return ra::CancelResponse::ACCEPT;
}

void Camera::srv_handle_accepted(const sptr<SrvGoalHandle> goal) {
  // Wait for calibration server
  if (!calib_client->wait_for_action_server(action_online_timeout)) {
    RCLCPP_ERROR(get_logger(), "Calibration server unavailable! Cancelling task...");
    goal->abort(result);
  }

  // Configuring the goal
  in_calibration = true;
  srv_handle = goal;
  CalibCamera::Goal calib_goal;
  CalibClient::SendGoalOptions opts;
  opts.goal_response_callback = std::bind(&Camera::accepted_callback, this, std::placeholders::_1);
  opts.feedback_callback =
      std::bind(&Camera::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  opts.result_callback = std::bind(&Camera::result_callback, this, std::placeholders::_1);
  calib_client->async_send_goal(calib_goal, opts);
}

///////////////////////////////////////////////////////////////////////////////
// Action Client (Receiving from calibration server)
///////////////////////////////////////////////////////////////////////////////

void Camera::accepted_callback(CltGoalHandle::ConstSharedPtr goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal rejected by the calibration server! It may be busy...");
    in_calibration = false;
    srv_handle->abort(result);
    srv_handle = nullptr;
    clt_handle = nullptr;
  }
  clt_handle = goal_handle;
}

void Camera::feedback_callback(CltGoalHandle::SharedPtr goal_handle,
                               const CalibCamera::Feedback::ConstSharedPtr feeback) {
  // Check if client is still valid
  if (srv_handle == nullptr) {
    RCLCPP_ERROR(get_logger(), "Client has disconnected during calibration! Stopping!");
    calib_client->async_cancel_goal(goal_handle);
    clt_handle = nullptr;
    in_calibration = false;
  }

  // Check for cancelling
  if (srv_handle->is_canceling()) {
    RCLCPP_WARN(get_logger(), "Client is cancelling the calibration!");
    calib_client->async_cancel_goal(goal_handle);
    clt_handle = nullptr;
    srv_handle = nullptr;
    in_calibration = false;
  }

  // Retransmit the data to the original client
  srv_feedback->n_step = feeback->n_step;
  srv_handle->publish_feedback(srv_feedback);
}

void Camera::result_callback(const CltGoalHandle::WrappedResult& calib_result) {
  switch (calib_result.code) {
    // If calib successful, load it internally
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Received calibration result!");
      cam_info.width = calib_result.result->cam_info.width;
      cam_info.height = calib_result.result->cam_info.height;
      cam_info.k = calib_result.result->cam_info.k;
      cam_info.d = calib_result.result->cam_info.d;
      cam_info.p = calib_result.result->cam_info.p;
      cam_info.r = calib_result.result->cam_info.r;
      cam_info.roi = calib_result.result->cam_info.roi;
      cam_info.distortion_model = calib_result.result->cam_info.distortion_model;
      cam_info.binning_x = calib_result.result->cam_info.binning_x;
      cam_info.binning_y = calib_result.result->cam_info.binning_y;
      cam_info.header = calib_result.result->cam_info.header;
      share_cam_info();
      srv_handle->succeed(result);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      srv_handle->abort(result);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      srv_handle->abort(result);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      srv_handle->abort(result);
      break;
  }

  // Clean request
  srv_handle = nullptr;
  clt_handle = nullptr;
  in_calibration = false;
}

}  // namespace raubase::cam