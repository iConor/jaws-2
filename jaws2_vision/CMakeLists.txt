cmake_minimum_required(VERSION 2.8.3)
project(jaws2_vision)

find_package(catkin REQUIRED COMPONENTS pointgrey_camera_driver camera_calibration cv_bridge)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
