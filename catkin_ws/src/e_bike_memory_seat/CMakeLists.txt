cmake_minimum_required(VERSION 3.0.2)
project(e_bike_memory_seat)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY actions
  FILES adjustSeatHeight.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp rospy std_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(ebmsActionServer src/server/EbmsActionServer.cpp)

add_library(canTransmit src/comm/canTransmit.cpp)

target_link_libraries(
  ebmsActionServer
  ${catkin_LIBRARIES}
  canTransmit
)

add_dependencies(
  ebmsActionServer
  ${e_bike_memory_seat_EXPORTED_TARGETS}
)
