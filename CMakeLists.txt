cmake_minimum_required(VERSION 3.0.2)
project(dmx_ros)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    message_runtime
    roscpp
    roslint
    std_msgs
)

add_message_files(DIRECTORY msg
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
    roscpp
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Roslint cpp
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()


add_executable(dmx_node src/dmx_node.cpp)
add_library(dmx485 src/dmx485.c)
add_dependencies(dmx_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dmx_node ${catkin_LIBRARIES} dmx485)

install(
  TARGETS
    dmx485
    dmx_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
