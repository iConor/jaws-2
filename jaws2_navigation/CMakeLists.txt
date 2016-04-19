cmake_minimum_required(VERSION 2.8.3)
project(jaws2_navigation)

find_package(catkin REQUIRED COMPONENTS roscpp jaws2_msgs tf)
find_package(Ceres REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

add_executable(thrust_mapper src/thrust_mapper.cpp)
target_link_libraries(thrust_mapper ${catkin_LIBRARIES} ${CERES_LIBRARIES})

add_executable(thrust_cal src/thrust_cal.cpp)
add_dependencies(thrust_cal jaws2_msgs_gencpp)
target_link_libraries(thrust_cal ${catkin_LIBRARIES})

add_executable(imu_transform src/imu_transform.cpp)
target_link_libraries(imu_transform ${catkin_LIBRARIES})

add_executable(orientation src/quat_to_rpy.cpp)
target_link_libraries(orientation ${catkin_LIBRARIES})
