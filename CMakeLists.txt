cmake_minimum_required(VERSION 3.0.2)
project(dvl_a50_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  DVL.msg
  DVLBeam.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Build publisher node
add_executable(publisher 
  src/publisher.cpp
)
add_dependencies(publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publisher
  ${catkin_LIBRARIES}
)

# Build subscriber node
add_executable(subscriber 
  src/subscriber.cpp
)
add_dependencies(subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(subscriber
  ${catkin_LIBRARIES}
)
