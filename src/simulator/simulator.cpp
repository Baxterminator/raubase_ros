#include "simulator/simulator.hpp"

#include "simulator/plugins/Motor.hpp"

namespace raubase::simu {

Simulator::Simulator(NodeOptions opts) : Node(NODE_NAME, opts) {
  // Declare plugins
  plugins.push_back(plugins::Motor::make());

  // Simulation
  simu_time = microseconds(declare_parameter("simu_ms", SIMU_TIME));
  simu_loop = create_wall_timer(simu_time, std::bind(&Simulator::nextIteration, this));

  // Activity checker
  activity = create_publisher<std_msgs::msg::Empty>(ACTIVITY_TOPIC, ACTIVITY_QOS);
}

}  // namespace raubase::simu

int main(int argc, char **argv) {
  init(argc, argv);

  // Initialize simulator
  auto simu = std::make_shared<raubase::simu::Simulator>(NodeOptions{});

  // Setup plugins
  simu->setup();

  // Run job
  while (ok()) {
    spin_some(simu);
  }

  // Cleaning
  simu->cleanup();

  shutdown();
  return 0;
}