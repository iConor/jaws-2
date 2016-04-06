cmake_minimum_required(VERSION 2.8.3)
project(jaws2_gazebo)

find_package(catkin REQUIRED COMPONENTS roscpp jaws2_msgs geometry_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(thrust_sim src/jaws2_gazebo.cpp)
add_dependencies(thrust_sim jaws2_msgs_gencpp)
target_link_libraries(thrust_sim ${catkin_LIBRARIES})
