cmake_minimum_required(VERSION 3.16)

project(hardware_pkg)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(motor_control_node
    src/motor_control_node.cpp
    src/pid.cpp
    src/pid_controller.cpp
    src/errorpid_controller.cpp
    src/motor_input.cpp
    src/gm6020_hardware.cpp
    src/lowpass_filter.cpp
)

target_include_directories(motor_control_node PUBLIC
    include

    ${CMAKE_CURRENT_SOURCE_DIR}/../../RMCS/rmcs_ws/src/rmcs_core/src

    ${CMAKE_CURRENT_SOURCE_DIR}/../../RMCS/rmcs_ws/build/rmcs_core/_deps/librmcs-src/core/include

    ${CMAKE_CURRENT_SOURCE_DIR}/../../RMCS/rmcs_ws/build/rmcs_core/_deps/librmcs-src/host/include

    ${CMAKE_CURRENT_SOURCE_DIR}/../../RMCS/rmcs_ws/install/include
)

target_link_libraries(motor_control_node
    ${CMAKE_CURRENT_SOURCE_DIR}/../../RMCS/rmcs_ws/build/rmcs_core/_deps/librmcs-build/host/liblibrmcs-sdk.a
    usb-1.0
    pthread
)