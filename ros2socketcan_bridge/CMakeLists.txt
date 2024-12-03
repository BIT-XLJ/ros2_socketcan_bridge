cmake_minimum_required(VERSION 3.5)
project(ros2socketcan_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(can_msgs REQUIRED)

set(TARGET_DEPENDENCE
  ament_cmake
  builtin_interfaces
  geometry_msgs
  rosidl_default_generators
  std_msgs
  rclcpp
  rmw
  rmw_implementation_cmake
  can_msgs
)

file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${source})
target_link_libraries(${PROJECT_NAME}  dl -lboost_system -pthread  -lcrypt -lm -lpthread)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)



# set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin ) 
# set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib ) 
# set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )

ament_target_dependencies(${PROJECT_NAME} ${TARGET_DEPENDENCE})
# ament_target_dependencies(ros2socketcan rclcpp std_msgs can_msgs)
# ament_export_targets(${PROJECT_NAME}_Targets HAS_LIBRARY_TARGET)
# ament_export_dependencies(${TARGET_DEPENDENCE})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})



install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  # INCLUDES DESTINATION include
)

# install(TARGETS ${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
