cmake_minimum_required(VERSION 3.0.2)
project(waterlinked_a50_ros_driver)

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
add_executable(a50_pub 
  src/publisher.cpp
)
add_dependencies(a50_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(a50_pub
  ${catkin_LIBRARIES}
)

# Build subscriber node
add_executable(a50_sub 
  src/subscriber.cpp
)
add_dependencies(a50_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(a50_sub
  ${catkin_LIBRARIES}
)

install(TARGETS a50_pub a50_sub
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
