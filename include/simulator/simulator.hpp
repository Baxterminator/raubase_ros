#ifndef RAUBASE_SIMULATOR
#define RAUBASE_SIMULATOR

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/detail/empty__struct.hpp>
#include <std_msgs/msg/empty.hpp>

#include "simulator/plugins/PluginInterface.hpp"

using namespace rclcpp;
using std::chrono::microseconds;

namespace raubase::simu {

/**
 * @brief Simulator class that mainly provide a closing for the several loops of the raubase
 * software when not using the Teensy Board.
 *
 */
class Simulator : public Node {
  // ==========================================================================
  //                                 Constants
  // ==========================================================================
  static constexpr const char* NODE_NAME{"simulator"};
  static constexpr int QOS{10};

  // Default values
  static constexpr int DEF_ODOM_FREQ{100};       //< Default odometry frequency
  static constexpr double DEF_GEAR_RATIO{19.0};  //< Default gear ratio
  static constexpr double DEF_WHEEL_D{0.146};    //< Default wheel diameter (in meters)
  static constexpr int DEF_TICK_PER_REV{68};     //< Default number of ticks per encoder revolution
  static constexpr double DEF_BASE{0.243};       //< Default wheel base width (in meters)
  static constexpr int MAX_TICK_CHANGE{1000};    //< Max acceptable tick change

  // ==========================================================================
  //                                 Methods
  // ==========================================================================
 public:
  // ----------------------------- Life Cycle ---------------------------------
  /**
   * @brief Instance a simulator and prepare for plugins.
   */
  Simulator(NodeOptions);

  /**
   * @brief Setup plugins and their components.
   */
  inline void setup() {
    ulong idx = 1, len = plugins.size();
    for (auto& plugin : plugins) {
      RCLCPP_WARN(get_logger(), "[%3zu/%3zu] Initializing plug-in %s", idx++, len,
                  plugin->getPluginName());
      plugin->setup(shared_from_this());
    }
  }

  /**
   * @brief Compute next iteration of the simulation.
   */
  inline void nextIteration() {
    static rclcpp::Time last_time = get_clock()->now();

    // Compute dt
    auto now = get_clock()->now();
    double dt = (now - last_time).seconds();
    last_time = now;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Running simulation ... [%.3f Hz]",
                         ((dt < 1E-5) ? -1 : 1 / dt));

    // Running update
    activity->publish(activity_msg);
    for (auto& plugin : plugins) plugin->update(dt);
  }

  /**
   * @brief Clean-up everything before stopping simulator.
   */
  inline void cleanup() {
    ulong idx = 1, len = plugins.size();
    for (auto& plugin : plugins) {
      RCLCPP_WARN(get_logger(), "[%3zu/%3zu] Cleaning-up plug-in %s", idx++, len,
                  plugin->getPluginName());
      plugin->cleanup();
    }
  }

  // ==========================================================================
  //                                    Members
  // ==========================================================================
 private:
  // ------------------------------- Simu Engine ------------------------------
  static constexpr int SIMU_TIME{50};
  microseconds simu_time;
  TimerBase::SharedPtr simu_loop;
  std::vector<plugins::PluginInterface::SharedPtr> plugins;

  static constexpr const char* ACTIVITY_TOPIC{"/teensy/activity"};
  static constexpr int ACTIVITY_QOS{10};
  std_msgs::msg::Empty activity_msg;
  Publisher<std_msgs::msg::Empty>::SharedPtr activity;
};

}  // namespace raubase::simu

#endif