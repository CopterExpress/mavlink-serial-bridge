cmake_minimum_required(VERSION 3.1.0)

set(CMAKE_C_FLAGS "-std=gnu99")

project(mavlink-serial-bridge C)

add_executable(mavlink-serial-bridge main.c)
