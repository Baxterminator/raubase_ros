#ifndef RAUBASE_SIMULATOR_MOTOR
#define RAUBASE_SIMULATOR_MOTOR

#include <chrono>
#include <raubase_msgs/msg/encoder_state.hpp>
#include <raubase_msgs/msg/motor_voltage.hpp>

#include "simulator/plugins/PluginInterface.hpp"
#include "teensy/proxy/encoder.hpp"
#include "teensy/proxy/motor.hpp"

using namespace raubase::teensy::proxy;
using raubase_msgs::msg::EncoderState;
using raubase_msgs::msg::MotorVoltage;

namespace raubase::simu::plugins {

/**
 * @brief Plugin for all motors-related simulation.
 */
struct Motor : public PluginInterface {
  static constexpr const char* PL_NAME{"MotorSimu"};

  /**
   * @brief Initializing the motor plugin for the simulation
   */
  Motor(){};

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
  static SharedPtr make() { return std::make_shared<Motor>(); };

 private:
  Clock::SharedPtr _clock;

  // --------------------------------- Motors ---------------------------------
  static constexpr const char* MOTOR_TOPIC = MotorProxy::SUBSCRIBING_TOPIC;
  static constexpr int MOTOR_QOS = MotorProxy::QOS;
  static constexpr double DEFAULT_KM{1.0};  //< Motor constant in rads/s/V
  double mot_righ_km, mot_left_km;
  MotorVoltage::SharedPtr _last_motor;
  Subscription<MotorVoltage>::SharedPtr sub_motor;

  // --------------------------------- Encoders -------------------------------
  static constexpr const char* ENC_TOPIC = EncoderProxy::PUBLISHING_TOPIC;
  static constexpr int ENC_QOS = EncoderProxy::QOS;
  EncoderState _enc_msg;
  TimerBase::SharedPtr enc_loop;
  Publisher<EncoderState>::SharedPtr pub_enc;
};

}  // namespace raubase::simu::plugins

#endif