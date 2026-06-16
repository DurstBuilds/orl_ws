cmake_minimum_required(VERSION 3.8)
project(psu_telemetry)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  scripts/psu_telemetry_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME psu_telemetry_node
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
