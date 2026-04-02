cmake_minimum_required(VERSION 3.8)
project(ik_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(trac_ik_lib REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(hand_publisher_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(trac_ik_node src/trac_ik_node.cpp)
ament_target_dependencies(trac_ik_node
  rclcpp std_msgs geometry_msgs sensor_msgs
  trac_ik_lib orocos_kdl
)

add_executable(cpp_heartbeat_node src/cpp_heartbeat_node.cpp)
ament_target_dependencies(cpp_heartbeat_node
  rclcpp std_msgs
)

add_executable(dual_finger_ik_node src/dual_finger_ik_node.cpp)
ament_target_dependencies(dual_finger_ik_node
  rclcpp sensor_msgs
  orocos_kdl kdl_parser
  hand_publisher_interfaces
  Eigen3
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

install(TARGETS
  trac_ik_node
  cpp_heartbeat_node
  dual_finger_ik_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
