cmake_minimum_required(VERSION 2.8.3)
project(sinusoid_path_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(PCL 1.8 REQUIRED)
find_package(PkgConfig)
find_package(Eigen3 3.3.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  roslaunch
)

## Declare a catkin package - necessary otherwise .cpp file won't be callable
## The INCLUDE_DIRS and LIBRARIES stuff is mostly for exporting purposes
catkin_package(
    INCLUDE_DIRS
      include
    LIBRARIES)

## Brings header files into the build environment
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR})

link_directories(${PCL_LIBRARY_DIRS})

add_library(sinusoid_path_planner src/sinusoid_path_planner.cpp)

target_link_libraries(sinusoid_path_planner ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
