cmake_minimum_required(VERSION 2.8.3)
project(jaws2_estimation)

find_package(catkin REQUIRED COMPONENTS roscpp tf geometry_msgs sensor_msgs imu_3dm_gx4)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
