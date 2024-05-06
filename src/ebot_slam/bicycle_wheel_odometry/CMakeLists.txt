PROJECT(bicycle_wheel_odometry)

CMAKE_MINIMUM_REQUIRED(VERSION 3.3)
# Require C++17
SET(CMAKE_BUILD_TYPE "Debug")
if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
	set(CMAKE_CXX_STANDARD 17)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  tf  
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)



#include_directories(${MRPT_INCLUDE_DIRS})
#MESSAGE( STATUS "MRPT_INCLUDE_DIRS: " ${MRPT_INCLUDE_DIRS})
#link_directories(${MRPT_LIBRARY_DIRS})
#MESSAGE( STATUS "MRPT_LIBRARY_DIRS: " ${MRPT_LIBS})



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
# INCLUDE_DIRS include
# LIBRARIES wheel_odometry
 CATKIN_DEPENDS nav_msgs geometry_msgs roscpp sensor_msgs std_msgs tf
 #DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(bicycle_wheel_odometry src/bicycle_wheel_odometry.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(bicycle_wheel_odometry
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${EIGEN_LIBRARIES}
)
