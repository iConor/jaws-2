cmake_minimum_required(VERSION 2.8.3)
project(jaws2_control)

find_package(catkin REQUIRED COMPONENTS roscpp tf control_toolbox sensor_msgs geometry_msgs imu_3dm_gx4)
find_package(Ceres REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

add_executable(attitude_controller src/attitude_controller.cpp)
target_link_libraries(attitude_controller ${catkin_LIBRARIES})

add_executable(thruster_controller src/thruster_controller.cpp)
target_link_libraries(thruster_controller ${catkin_LIBRARIES} ${CERES_LIBRARIES})
