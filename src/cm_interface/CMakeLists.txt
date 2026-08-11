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
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(motor_node_continuous src/motor_node_continuous.cpp)
target_include_directories(motor_node_continuous PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motor_node_continuous rclcpp std_msgs motor_interfaces)

add_executable(can_gateway_node src/can_gateway_node.cpp)
target_include_directories(can_gateway_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(can_gateway_node rclcpp std_msgs motor_interfaces)

add_executable(motor_unwrapper_node src/motor_unwrapper_node.cpp)
ament_target_dependencies(motor_unwrapper_node rclcpp std_msgs motor_interfaces)

add_executable(joint_translator_node src/joint_translator_node.cpp)
target_include_directories(joint_translator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(joint_translator_node rclcpp std_msgs motor_interfaces)

install(TARGETS
  motor_node_continuous
  can_gateway_node
  motor_unwrapper_node
  joint_translator_node
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

install(PROGRAMS
  scripts/joint_despos_command.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_despos_command
)

install(PROGRAMS
  scripts/joint_position_sequence.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_position_sequence_node
)

install(PROGRAMS
  scripts/can0_up.sh
  scripts/boom_stack_stop.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
