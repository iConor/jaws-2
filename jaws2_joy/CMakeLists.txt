cmake_minimum_required(VERSION 2.8.3)
project(jaws2_joy)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jaws2_joy
#  CATKIN_DEPENDS geometry_msgs joy roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(accel_error src/accel_error.cpp)
add_executable(airplane src/airplane.cpp)

target_link_libraries(accel_error
  ${catkin_LIBRARIES}
)
target_link_libraries(airplane
  ${catkin_LIBRARIES}
)
