cmake_minimum_required(VERSION 3.25)

project(CAVR
  VERSION 0.1.0
  DESCRIPTION "ROS-free C++20 record and replay framework for synchronized industrial robot and camera data"
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(CAVR_BUILD_TESTS "Build CAVR tests" ON)
option(CAVR_BUILD_APPS "Build CAVR command-line applications" ON)
option(CAVR_BUILD_ADAPTERS "Build CAVR open-source adapters" ON)

add_subdirectory(libs/core)
add_subdirectory(libs/time)
add_subdirectory(libs/machine)
add_subdirectory(libs/frame_graph)
add_subdirectory(libs/adapter_sdk)
add_subdirectory(libs/storage_mcap)
add_subdirectory(libs/record)
add_subdirectory(libs/catalog)
add_subdirectory(libs/replay)
add_subdirectory(libs/transport)
add_subdirectory(libs/calibration)
add_subdirectory(libs/fault_injection)
add_subdirectory(libs/validation)
add_subdirectory(libs/runtime)
add_subdirectory(libs/visualization)

if(CAVR_BUILD_ADAPTERS)
  add_subdirectory(adapters/file_camera)
  add_subdirectory(adapters/mock_camera)
  add_subdirectory(adapters/opencv_camera)
  add_subdirectory(adapters/file_robot)
  add_subdirectory(adapters/mock_robot)
  add_subdirectory(adapters/generic_tcp_robot)
  add_subdirectory(adapters/robodk)
endif()

if(CAVR_BUILD_APPS)
  add_subdirectory(apps/cavr-studio)
  add_subdirectory(apps/cavr-play)
  add_subdirectory(apps/cavr-record)
  add_subdirectory(apps/cavr-inspect)
  add_subdirectory(apps/cavr-validate)
  add_subdirectory(apps/cavr-convert)
  add_subdirectory(apps/cavr-robotd)
endif()

if(CAVR_BUILD_TESTS)
  include(CTest)
  add_subdirectory(tests)
endif()
