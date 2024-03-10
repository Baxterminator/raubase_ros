#include "simulator/simulator.hpp"

#include "simulator/plugins/Motor.hpp"

namespace raubase::simu {

Simulator::Simulator(NodeOptions opts) : Node(NODE_NAME, opts) {
  // Declare robot
  auto robot = TwoWheeledRoverKinematics::make(
      Wheel(declare_parameter("right_wheel_diameter_m", DEF_WHEEL_D),
            declare_parameter("right_gear_ratio", DEF_GEAR_RATIO),
            declare_parameter("right_tick_per_rev", DEF_TICK_PER_REV)),
      Wheel(declare_parameter("left_wheel_diameter_m", DEF_WHEEL_D),
            declare_parameter("left_gear_ratio", DEF_GEAR_RATIO),
            declare_parameter("left_tick_per_rev", DEF_TICK_PER_REV)),
      MAX_TICK_CHANGE, declare_parameter("base_width_m", DEF_BASE));

  // Declare plugins
  plugins.push_back(plugins::Motor::make(robot));

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