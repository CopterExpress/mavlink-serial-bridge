cmake_minimum_required(VERSION 3.1.0)

set(CMAKE_C_FLAGS "-std=gnu99")

project(mavlink-serial-bridge C)

set(VERSION_MAJOR 0)
set(VERSION_MINOR 8)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DVERSION_MAJOR=${VERSION_MAJOR} -DVERSION_MINOR=${VERSION_MINOR}")

if((NOT EXISTS "${PROJECT_SOURCE_DIR}/lib/mavlink/v2.0/protocol.h") OR (NOT EXISTS "${PROJECT_SOURCE_DIR}/lib/c-ringbuf/ringbuf.c"))
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

find_package(PkgConfig REQUIRED)

pkg_check_modules(YAML REQUIRED libcyaml)

add_executable(mavlink-serial-bridge main.c config.c ${PROJECT_SOURCE_DIR}/lib/c-ringbuf/ringbuf.c)

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address,undefined,leak")
set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address,undefined,leak")

target_include_directories(mavlink-serial-bridge PUBLIC ${YAML_INCLUDE_DIRS} ${PROJECT_BINARY_DIR} ${PROJECT_SOURCE_DIR}/lib/c-ringbuf)
target_link_libraries(mavlink-serial-bridge ${YAML_LIBRARIES})

configure_file("${PROJECT_SOURCE_DIR}/mavlink-serial-bridge@.service.cmake"
    "${PROJECT_BINARY_DIR}/mavlink-serial-bridge@.service")

install(TARGETS mavlink-serial-bridge DESTINATION bin)
install(FILES "${PROJECT_BINARY_DIR}/mavlink-serial-bridge@.service" DESTINATION 
    "/lib/systemd/system")
install(FILES "${PROJECT_SOURCE_DIR}/config.yaml" DESTINATION 
    "/etc/mavlink-serial-bridge/" RENAME "example.yaml")
