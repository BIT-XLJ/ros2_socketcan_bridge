cmake_minimum_required(VERSION 3.8)
project(ros2_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2socketcan_bridge REQUIRED)
find_package(can_msgs REQUIRED)

set(TARGET_DEPENDENCE
  ament_cmake
  rclcpp
  std_msgs
  can_msgs
  ros2socketcan_bridge
)

include_directories(
  include
)

add_executable(socketcan_test src/can_test_node.cpp)

target_include_directories(socketcan_test PRIVATE 
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           )

ament_target_dependencies(socketcan_test ${TARGET_DEPENDENCE})


install(
  TARGETS socketcan_test  # 需要安装的目标
  DESTINATION lib/${PROJECT_NAME}  # 安装路径
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
