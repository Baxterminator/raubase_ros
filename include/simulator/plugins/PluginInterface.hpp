#ifndef RAUBASE_SIMULATOR_PLUGIN
#define RAUBASE_SIMULATOR_PLUGIN

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "common/utils/types.hpp"


using namespace rclcpp;
using std::chrono::microseconds;
using std::chrono::milliseconds;

namespace raubase::simu::plugins {

/**
 * @brief Definition of the interface of the plugins for the simulator.
 */
struct PluginInterface {
  typedef sptr<PluginInterface> SharedPtr;

  /**
   * @brief Get the plugin name
   *
   * @return const char*
   */
  virtual const char* getPluginName() const = 0;

  /**
   * @brief Setup all plugins requirements (pub / sub / internal values)
   */
  virtual void setup(Node::SharedPtr) = 0;

  /**
   * @brief Loop update for the plugin.
   *
   * @param dt the time since the last loop iteration
   */
  virtual void update(double dt) = 0;

  /**
   * @brief Cleaning up all components of this plugins before exiting.
   */
  virtual void cleanup() = 0;
};

}  // namespace raubase::simu::plugins

#endif