#include "can_test_node.h"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  
  auto can_test_node = std::make_shared<MyNode>();

  auto ros2_socket_can_node = std::make_shared<ros2socketcan>("can1");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 5);
  try {
    executor.add_node(ros2_socket_can_node);
    executor.add_node(can_test_node);
    executor.spin();
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(can_test_node->get_logger(), e.what() << '\n');
  }
  rclcpp::shutdown();
  return 0;
}

