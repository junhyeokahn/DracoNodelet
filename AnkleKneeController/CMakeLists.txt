cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

SET(CMAKE_CXX_FLAGS "-std=c++14")
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(PkgConfig)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

FILE(GLOB_RECURSE srcs ${PROJECT_SOURCE_DIR}/src/AnkleKneeInterface.cpp)

rosbuild_add_library(AnkleKneeController SHARED ${srcs})
target_link_libraries(AnkleKneeController data_manager
                                          utils)
