cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

SET(CMAKE_CXX_FLAGS "-std=c++11")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(PkgConfig)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

set(THIS_COM ${PROJECT_SOURCE_DIR})

rosbuild_add_library(data_manager SHARED
  include/data_manager/DataManager.hpp
  src/DataManager.cpp)

target_link_libraries(data_manager utils)
target_link_libraries (data_manager ${PYTHON_LIBRARIES} )

rosbuild_add_executable(run_data_saver
  src/DataReceiver.cpp
  include/data_manager/DataSave.hpp
  src/DataSave.cpp)

target_link_libraries(run_data_saver utils)

SET(DATA_LOCATION ${PROJECT_SOURCE_DIR}/)
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/include/data_manager/dm_config.h.cmake ${PROJECT_SOURCE_DIR}/include/data_manager/DataManager_Configuration.h)
