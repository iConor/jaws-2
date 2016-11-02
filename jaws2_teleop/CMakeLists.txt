cmake_minimum_required(VERSION 2.8.3)
project(jaws2_teleop)

find_package(catkin REQUIRED COMPONENTS geometry_msgs joy roscpp)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
