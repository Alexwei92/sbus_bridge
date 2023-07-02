cmake_minimum_required(VERSION 3.0.2)
project(sbus_bridge)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  SbusRosMessage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sbus_bridge
  src/sbus_bridge.cpp
  src/sbus_serial_port.cpp
  src/sbus_msg.cpp
)

target_link_libraries(sbus_bridge ${catkin_LIBRARIES})
