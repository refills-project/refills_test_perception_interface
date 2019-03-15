cmake_minimum_required(VERSION 2.8.3)
project(refills_test_perception_interface)

find_package(catkin REQUIRED refills_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES refills_test_intel_perception_interface
  CATKIN_DEPENDS refills_msgs
# DEPENDS other libs here 
)
