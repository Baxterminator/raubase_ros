#include <chrono>
#include <cstring>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <robotbot_msgs/srv/detail/ask_camera_image__struct.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace rclcpp;
using robotbot_msgs::srv::AskCameraImage;
using namespace std::chrono_literals;

void test_camera(rclcpp::Node::SharedPtr &node,
                 rclcpp::executors::SingleThreadedExecutor::SharedPtr &exec) {
  auto client = node->create_client<robotbot_msgs::srv::AskCameraImage>("/camera/get_image");
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>("/testing/img", 10);

  auto req = std::make_shared<AskCameraImage::Request>();
  while (ok()) {
    while (!client->wait_for_service(1s)) {
      if (!ok()) {
        RCLCPP_ERROR(node->get_logger(), "ROS Crashed!");
        return;
      }
      RCLCPP_INFO(node->get_logger(), "Waiting for service to got online ...");
    }

    auto start = std::chrono::high_resolution_clock::now();
    auto result = client->async_send_request(req);
    if (exec->spin_until_future_complete(result) == rclcpp::FutureReturnCode::SUCCESS) {
      auto end = std::chrono::high_resolution_clock::now();

      auto r = result.get();
      publisher->publish(r->image);

      RCLCPP_INFO(node->get_logger(), "Took %.2f ms. Got %ux%u image with encoding %s (%d)",
                  (end - start).count() * 1E-6, r->image.width, r->image.height,
                  r->image.encoding.c_str(),
                  r->image.data.size() == (r->image.width * r->image.height * 3));
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to call service ...");
    }
  }
}

int main(int argc, char **argv) {
  init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = std::make_shared<Node>("testing", NodeOptions{});
  exec->add_node(node);
  auto test_to_do = node->declare_parameter("what", "camera");

  if (strcmp(test_to_do.c_str(), "camera") == 0) test_camera(node, exec);

  shutdown();
  return 0;
}