cmake_minimum_required(VERSION 3.8)
project(nhk2026_r1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robomas_interfaces REQUIRED)

add_executable(main_node src/main_node.cpp)
ament_target_dependencies(main_node
  rclcpp
  std_msgs
  sensor_msgs
  robomas_interfaces
)

install(TARGETS
  main_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(single_homing_node src/single_homing_node.cpp)
ament_target_dependencies(single_homing_node
  rclcpp
  std_msgs
  robomas_interfaces
)

install(TARGETS
  single_homing_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(single_motor_joy_node src/single_motor_joy_node.cpp)
ament_target_dependencies(single_motor_joy_node
  rclcpp
  std_msgs
  sensor_msgs
  robomas_interfaces
)

install(TARGETS
  single_motor_joy_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(current_vel_limit_node src/current_vel_limit_node.cpp)
ament_target_dependencies(current_vel_limit_node
  rclcpp
  std_msgs
  sensor_msgs
  robomas_interfaces
)

install(TARGETS
  current_vel_limit_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(simple_velocity_node src/simple_velocity_node.cpp)
ament_target_dependencies(simple_velocity_node
  rclcpp
  std_msgs
  sensor_msgs
  robomas_interfaces
)

install(TARGETS
  simple_velocity_node
  DESTINATION lib/${PROJECT_NAME}
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
