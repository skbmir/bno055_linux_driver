cmake_minimum_required(VERSION 2.8.3)
project(bno055_linux_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=gnu++11 -fpermissive)
set(CMAKE_C_FLAGS "-std=c99")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include include/bno055_linux_driver
  LIBRARIES bno055
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/bno055_linux_driver
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(bno055 src/bno055.c)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(bno055_node src/bno055_node.cpp)
add_executable(bno055_calibrate src/bno055_calibrate.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(bno055_node
   bno055
   ${catkin_LIBRARIES}
)
target_link_libraries(bno055_calibrate
    bno055
   ${catkin_LIBRARIES}
)