cmake_minimum_required(VERSION 2.8.3)
project(dmx_ros)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    message_runtime
    rospy
    std_msgs
)

catkin_python_setup()

add_message_files(
  DIRECTORY msg
  FILES
    Dmx.msg
)

add_service_files(DIRECTORY srv
  FILES
    GetChannel.srv
    SetChannel.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)

install(PROGRAMS
  src/driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
