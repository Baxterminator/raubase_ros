#ifndef RAUBASE_SIMULATOR_MOTOR
#define RAUBASE_SIMULATOR_MOTOR

#include <chrono>
#include <raubase_msgs/msg/cmd_motor_voltage.hpp>
#include <raubase_msgs/msg/data_encoder.hpp>
#include <rclcpp/logger.hpp>

#include "common/robot/kinematics.hpp"
#include "simulator/plugins/PluginInterface.hpp"
#include "teensy/proxy/encoder.hpp"
#include "teensy/proxy/motor.hpp"

using namespace raubase::teensy::proxy;
using raubase::kinematics::TwoWheeledRoverKinematics;
using raubase::kinematics::Wheel;
using raubase_msgs::msg::CmdMotorVoltage;
using raubase_msgs::msg::DataEncoder;

namespace raubase::simu::plugins {

/**
 * @brief Plugin for all motors-related simulation.
 */
struct Motor : public PluginInterface {
  static constexpr const char* PL_NAME{"MotorSimu"};

  /**
   * @brief Initializing the motor plugin for the simulation
   */
  Motor(sptr<TwoWheeledRoverKinematics> kine) : robot(kine){};

  inline const char* getPluginName() const override { return PL_NAME; }

  /**
   * @brief Setup all plugins requirements (pub / sub / internal values)
   */
  void setup(rclcpp::Node::SharedPtr) override;

  /**
   * @brief Loop update for the plugin.
   *
   * @param dt the time since the last loop iteration
   */
  void update(double dt) override;

  /**
   * @brief Cleaning up all components of this plugins before exiting.
   */
  void cleanup() override;

  /**
   * @brief Create a shared pointer for instanciating this plugin.
   * @return a shared ptr to this plugin
   */
  static SharedPtr make(sptr<TwoWheeledRoverKinematics> rob) {
    return std::make_shared<Motor>(rob);
  };

 private:
  Clock::SharedPtr _clock;
  Logger _logger = get_logger(PL_NAME);
  sptr<kinematics::TwoWheeledRoverKinematics> robot;

  // --------------------------------- Motors ---------------------------------
  static constexpr const char* MOTOR_TOPIC = MotorProxy::SUBSCRIBING_TOPIC;
  static constexpr int MOTOR_QOS = MotorProxy::QOS;
  static constexpr double DEFAULT_KM{40.067609};  //< Motor constant in rads/s/V
  double mot_righ_km, mot_left_km;
  CmdMotorVoltage::SharedPtr _last_motor;
  Subscription<CmdMotorVoltage>::SharedPtr sub_motor;

  // --------------------------------- Encoders -------------------------------
  static constexpr const char* ENC_TOPIC = EncoderProxy::PUBLISHING_TOPIC;
  static constexpr int ENC_QOS = EncoderProxy::QOS;
  double right_wheel = 0, left_wheel = 0;
  DataEncoder _enc_msg;
  TimerBase::SharedPtr enc_loop;
  Publisher<DataEncoder>::SharedPtr pub_enc;
};

}  // namespace raubase::simu::plugins

#endif