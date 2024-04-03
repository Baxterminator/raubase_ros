#include <algorithm>

#include "common/math/fixed_pid_control.hpp"
#include "common/math/math.hpp"
#include "common/math/pid_control.hpp"
#include "common/utils/rmw.hpp"
#include "common/utils/types.hpp"
#include "common/utils/utime.hpp"
#include "controller/edge_controller.hpp"

namespace raubase::control {

LineFollower::LineFollower(NodeOptions opts) : Node(NODE_NAME, opts) {
  // Declare parameters

  setup_controller();
  max_turn_rate = declare_parameter(Params::MAX_TR, Default::MAX_TR);
  n_sensors = declare_parameter(Params::N_SENSORS, Default::N_SENSORS);
  half_n_sensors = float(n_sensors - 1) / 2.;
  float width = declare_parameter(Params::WIDTH, Default::WIDTH);
  btwn_m = width / (n_sensors - 1);
  center_m = width / 2.0;

  white_calibration = declare_parameter(Params::WHITE_CALIB, calib(n_sensors, Default::WHITE));
  black_calibration = declare_parameter(Params::BLACK_CALIB, calib(n_sensors, Default::BLACK));
  white_threshold = declare_parameter(Params::WHITE_THRES, Default::W_THRESHOLD);
  check_calib();
  if (!calib_valid) RCLCPP_ERROR(get_logger(), "Invalid calibration for line controller!");

  // Register ROS components
  move_cmd.move_type = CmdMove::CMD_V_TR;
  normalized.data.resize(n_sensors);
  move_pub = create_publisher<CmdMove>(Topics::PUB_CMD, QOS);
  sensor_sub = create_subscription<DataLineSensor>(Topics::SUB_LINE, QOS,
                                                   [this](DataLineSensor::SharedPtr msg) {
                                                     last_data = msg;
                                                     last_data_has_been_used = false;
                                                     if (consuming) loop();
                                                   });
  stop_sub =
      create_subscription<DataGPIO>(Topics::SUB_STOP, QOS, [this](DataGPIO::ConstSharedPtr msg) {
        if (msg->value)
          last_cmd = []() {
            auto cmd = std::make_shared<CmdLineFollower>();
            cmd->speed = 0;
            cmd->offset = 0;
            return cmd;
          }();
      });
  ref_sub = create_subscription<CmdLineFollower>(
      Topics::SUB_REF, QOS, [this](CmdLineFollower::SharedPtr ref) { last_cmd = ref; });
  result_pub = create_publisher<ResultEdge>(Topics::PUB_RESULT, QOS);
  declare_controller();
  normalized_pub = create_publisher<DataLineSensor>(Topics::PUB_NORMALIZED, QOS);

  controller_state = create_subscription<StateVelocityController>(
      Topics::SUB_CONTROLLER_STATE, QOS,
      [this](StateVelocityController::SharedPtr msg) { last_control_state = msg; });

  sensor_config = create_publisher<SetLineSensorConfig>(Topics::PUB_LINE_SET, 10);
  config.high_power = true;
  config.on = true;
  config.white = true;
}

///////////////////////////////////////////////////////////////////////////////

void LineFollower::setup_controller() {
  int freq = declare_parameter(Params::PID_FREQ, Default::FREQ);

  // If controller works on "as soon as we have the odometry"
  if (freq == -1) {
    RCLCPP_INFO(get_logger(), "Launching edge controller unit in consuming mode");
    consuming = true;
    pid = math::PILeadController::make(declare_parameter(Params::PID_KP, Default::PID_KP),
                                       declare_parameter(Params::PID_TD, Default::PID_TD),
                                       declare_parameter(Params::PID_AD, Default::PID_AD),
                                       declare_parameter(Params::PID_TI, Default::PID_TI));
  }
  // Else the controller is at a fixed speed
  else {
    RCLCPP_INFO(get_logger(), "Launching edge controller unit with a period of %zuµs",
                (long)std::floor(1.0 / freq * 1E6));
    pid = math::FixedPILeadController::make(declare_parameter(Params::PID_KP, Default::PID_KP),
                                            declare_parameter(Params::PID_TD, Default::PID_TD),
                                            declare_parameter(Params::PID_AD, Default::PID_AD),
                                            declare_parameter(Params::PID_TI, Default::PID_TI),
                                            1.0 / ((float)freq));

    fixed_loop = create_wall_timer(microseconds((long)std::floor(1. / freq * 1E6)),
                                   std::bind(&LineFollower::loop, this));
  }
}

void LineFollower::declare_controller() {
  input_declaration = create_publisher<SetControllerInput>(Topics::PUB_DECLARE, TRANSIENT_QOS);
  SetControllerInput declaration;
  declaration.input = SetControllerInput::EDGE;
  input_declaration->publish(declaration);
}

///////////////////////////////////////////////////////////////////////////////

void LineFollower::check_calib() {
  ulong white_size = white_calibration.size();
  ulong black_size = black_calibration.size();

  // Check vector sizes
  if (calib_valid = (white_size == black_size) && (white_size = n_sensors); !calib_valid) return;

  // Check calibration spacing
  factor.reserve(n_sensors);
  for (ulong i = 0; i < n_sensors; i++) {
    calib_valid = (white_calibration[i] - black_calibration[i]) > MIN_DIFF_W_B;
    factor[i] = 1000.0 / (white_calibration[i] - black_calibration[i]);
    if (!calib_valid) return;
  }
}

///////////////////////////////////////////////////////////////////////////////

void LineFollower::compute_edge() {
  result.valid_edge = false;

  // Iterate of the values of the sensor to find the edge
  ulong i, j;
  bool found_left = false, found_right = false;
  for (i = 0; (i < n_sensors) && (!found_left || !found_right); i++) {
    j = n_sensors - 1 - i;

    // Compute normalized value (compute only once)
    if (i < j) {
      normalized.data[i] = (last_data->data[i] - black_calibration[i]) * factor[i];
      normalized.data[j] = (last_data->data[j] - black_calibration[j]) * factor[j];
    }

    // Look for left edge
    if (!found_left && normalized.data[i] > white_threshold) {
      result.left_edge = (i == 0)
                             ? 0
                             : (float(i) + float(white_threshold - normalized.data[i - 1]) /
                                               float(normalized.data[i] - normalized.data[i - 1]));
      found_left = true;
    }

    // Look for right edge
    if (!found_right && normalized.data[j] > white_threshold) {
      result.right_edge = (j == n_sensors - 1)
                              ? n_sensors - 1
                              : (float(j) - float(white_threshold - normalized.data[j + 1]) /
                                                float(normalized.data[j] - normalized.data[j + 1]));
      found_right = true;
    }
  }

  // Found edge if found left or right
  if (result.valid_edge = found_left || found_right; !result.valid_edge) {
    result.left_edge = half_n_sensors;
    result.right_edge = half_n_sensors;
  }

  // Scale to meters (positive is left)
  result.left_edge = center_m - (result.left_edge * btwn_m);
  result.right_edge = center_m - (result.right_edge * btwn_m);

  normalized.stamp = get_clock()->now();
  result.width = result.left_edge - result.right_edge;
  result_pub->publish(result);
  normalized_pub->publish(normalized);
}

///////////////////////////////////////////////////////////////////////////////

void LineFollower::update_controller(double dt) {
  float u;

  if (result.valid_edge) {
    u = pid->update(dt, last_cmd->offset,
                    ((last_cmd->follow) ? result.right_edge : result.left_edge), limited);
    limited = std::fabs(u) > max_turn_rate || last_control_state->voltage_saturation ||
              last_control_state->turnrate_saturation;
    move_cmd.turn_rate = math::saturate(u, max_turn_rate);
    move_cmd.velocity = last_cmd->speed;
  } else {
    move_cmd.turn_rate = 0.0;
    move_cmd.velocity = 0.0;
    limited = last_control_state->voltage_saturation || last_control_state->turnrate_saturation;
  }

  move_pub->publish(move_cmd);
}

///////////////////////////////////////////////////////////////////////////////

void LineFollower::loop() {
  static UTime loop_clock("now");

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), THROTTLE_DUR, "Edge valid %d", calib_valid);
  // If calib not valid, don't continue
  if (!calib_valid) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), THROTTLE_DUR, "Calibration is not good!");
    return;
  }

  sensor_config->publish(config);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), THROTTLE_DUR, "Edge loop");
  // Initialize last enc
  if (last_data_used == nullptr) {
    last_data_used = last_data;
    return;
  }

  // Sanity check for last_enc value
  if (last_data == nullptr || last_cmd == nullptr) return;

  // Prevent from reusing the same one
  if (last_data_has_been_used) return;

  // Compute edge
  compute_edge();
  update_controller(loop_clock.getTimePassed());
  last_data_has_been_used = true;

  loop_clock.now();
}

}  // namespace raubase::control

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<raubase::control::LineFollower>(rclcpp::NodeOptions{});

  while (rclcpp::ok()) {
    rclcpp::spin(node);
  }

  rclcpp::shutdown();
  return 0;
}
