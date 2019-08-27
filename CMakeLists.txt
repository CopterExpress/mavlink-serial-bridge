cmake_minimum_required(VERSION 3.1.0)

set(CMAKE_C_FLAGS "-std=gnu99")

project(mavlink-serial-bridge C)

if(NOT EXISTS "${PROJECT_SOURCE_DIR}/lib/mavlink/v2.0")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

if(NOT DEFINED MAVLINK_DIALECT)
    set(MAVLINK_DIALECT "ardupilotmega")
    message(WARNING "MAVLink dialect is not set. Using ardupilotmega by default!")
endif()

configure_file(
  "${PROJECT_SOURCE_DIR}/mavlink_dialect.h.in"
  "${PROJECT_BINARY_DIR}/mavlink_dialect.h"
)

add_executable(mavlink-serial-bridge main.c)

target_include_directories(mavlink-serial-bridge PUBLIC ${PROJECT_BINARY_DIR})
