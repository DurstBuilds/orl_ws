cmake_minimum_required(VERSION 3.8)
project(cm_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(motor_interfaces REQUIRED)

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

# required for can_test_node
add_executable(can_test_node src/can_test_node.cpp)
ament_target_dependencies(can_test_node rclcpp std_msgs)

# required for motor_node
add_executable(motor_node src/motor_node.cpp)
ament_target_dependencies(motor_node rclcpp std_msgs motor_interfaces)

# continuous delta-position MIT firmware
add_executable(motor_node_continuous src/motor_node_continuous.cpp)
target_include_directories(motor_node_continuous PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motor_node_continuous rclcpp std_msgs motor_interfaces)

add_executable(motor_unwrapper_node src/motor_unwrapper_node.cpp)
ament_target_dependencies(motor_unwrapper_node rclcpp std_msgs motor_interfaces)

add_executable(joint_translator_node src/joint_translator_node.cpp)
target_include_directories(joint_translator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(joint_translator_node rclcpp std_msgs motor_interfaces)

# required for terminal_position_publisher
add_executable(terminal_position_publisher src/terminal_position_publisher.cpp)
ament_target_dependencies(terminal_position_publisher rclcpp std_msgs)

# required for terminal_motor_command
add_executable(terminal_motor_command src/terminal_motor_command.cpp)
ament_target_dependencies(terminal_motor_command rclcpp std_msgs motor_interfaces)

install(TARGETS
  can_test_node
  motor_node
  motor_node_continuous
  motor_unwrapper_node
  joint_translator_node
  terminal_position_publisher
  terminal_motor_command
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/keyboard_command.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_command
)

install(PROGRAMS
  scripts/joystick_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joystick_control_node
)

install(PROGRAMS
  scripts/boom_joystick_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME boom_joystick_control_node
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
