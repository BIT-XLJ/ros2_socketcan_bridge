#include "can_test_node.h"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  
  auto can_test_node = std::make_shared<CanNode>("can0");

  auto ros2_socket_can_node = std::make_shared<ros2socketcan>("can0","can0_node");
  
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 5);
  try 
  {
    executor.add_node(ros2_socket_can_node);
    executor.add_node(can_test_node);
    can_test_node->init_motor(3,false);
    can_test_node->enable_all_motors();
    can_test_node->mit_mode_send(3, 0.0, 0.0, 100 * M_PI, 2.00, 1.0);  // 延迟后执行发送
    executor.spin();
  } 
  catch (const std::exception &e) 
  {
    RCLCPP_ERROR_STREAM(can_test_node->get_logger(), e.what() << '\n');
  }
  rclcpp::shutdown();
  return 0;
}

