cmake_minimum_required(VERSION 3.1.0)

set(CMAKE_C_FLAGS "-std=gnu99")

project(swarm-drone-bridge C)

add_executable(swarm-drone-bridge main.c)

