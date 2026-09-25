cmake_minimum_required(VERSION 3.8)

project(hardware_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 使用 C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 头文件目录
include_directories(
  include
)

# PID
add_library(pid
  src/pid.cpp
)

target_include_directories(pid
  PUBLIC
    include
)

# PID Controller
add_library(pid_controller
  src/pid_controller.cpp
)

target_include_directories(pid_controller
  PUBLIC
    include
)

target_link_libraries(pid_controller
  PUBLIC
    pid
)

# Error PID Controller
add_library(errorpid_controller
  src/errorpid_controller.cpp
)

target_include_directories(errorpid_controller
  PUBLIC
    include
)

target_link_libraries(errorpid_controller
  PUBLIC
    pid
)

# Motor Input
add_library(motor_input
  src/motor_input.cpp
)

target_include_directories(motor_input
  PUBLIC
    include
)

# GM6020 Hardware
add_library(gm6020_hardware
  src/gm6020_hardware.cpp
)

target_include_directories(gm6020_hardware
  PUBLIC
    include
)

target_link_libraries(gm6020_hardware
  PUBLIC
    motor_input
)

# CAN Interface
add_library(can_interface
  src/can_interface.cpp
)

target_include_directories(can_interface PUBLIC include)

# Low Pass Filter
add_library(lowpass_filter
  src/lowpass_filter.cpp
)

target_include_directories(lowpass_filter PUBLIC include)

# Motor Control Node
add_executable(motor_control_node
  src/motor_control_node.cpp
)

target_include_directories(motor_control_node PUBLIC include)

target_link_libraries(motor_control_node
  pid
  pid_controller
  errorpid_controller
  motor_input
  gm6020_hardware
  can_interface
  lowpass_filter
)