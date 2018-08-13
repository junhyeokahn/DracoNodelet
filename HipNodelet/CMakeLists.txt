cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(PkgConfig)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-std=c++14")

rosbuild_add_library(HipNodelet src/hip_nodelet.cpp)
target_link_libraries(HipNodelet apptronik_ros_utils
                                 ApptronikSystem)
target_link_libraries(HipNodelet HipController)
