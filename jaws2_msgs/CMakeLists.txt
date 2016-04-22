cmake_minimum_required(VERSION 2.8.3)
project(jaws2_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(FILES Pwm.msg PwmStamped.msg ServoCommand.msg ServoCommandStamped.msg ServoState.msg ServoStateStamped.msg Thrust.msg ThrustStamped.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

#include_directories(${catkin_INCLUDE_DIRS})
