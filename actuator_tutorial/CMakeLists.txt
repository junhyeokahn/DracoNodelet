cmake_minimum_required(VERSION 3.0.2)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif(NOT CMAKE_BUILD_TYPE)

project(actuator_tutorial)

find_package(catkin REQUIRED COMPONENTS cortex_utils message_generation message_runtime nodelet pluginlib roscpp roslib rt_utils tf2_eigen nodelet)
find_package(Boost QUIET REQUIRED COMPONENTS system thread filesystem)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cortex_utils message_generation message_runtime nodelet pluginlib roscpp roslib rt_utils tf2_eigen
  )

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++14 -fPIC -Wno-unused-result -Wno-overflow -Dunix -rdynamic")

FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(EIGEN3 REQUIRED eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/actuator_nodelet.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
