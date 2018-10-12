cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_CXX_FLAGS "-std=c++14")

FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(EIGEN3 REQUIRED eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

include_directories("/usr/local/include/DynaCoRE")
link_directories("/usr/local/lib")

rosbuild_add_library(DracoWalkingNodelet src/DracoWalkingNodelet.cpp)
target_link_libraries(DracoWalkingNodelet rt
                                          apptronik_ros_utils
                                          DracoBip_Controller 
                                          dynacore_utils 
                                          dynacore_param_handler)
