cmake_minimum_required(VERSION 3.5)
project(robot_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

add_library(robot_hardware SHARED
  src/omni_wheel_hardware.cpp
  src/vex_comms.cpp
)

target_include_directories(
  robot_hardware
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  robot_hardware
  hardware_interface
  controller_manager
  serial
  rclcpp
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

install(
  TARGETS robot_hardware
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY controllers launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES robot_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(robot_hardware)
ament_export_dependencies(
  hardware_interface
  serial
  controller_manager
  rclcpp
  pluginlib
)

ament_package()