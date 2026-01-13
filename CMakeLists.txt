cmake_minimum_required(VERSION 3.8)
project(dvl_a50_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json REQUIRED)

# 2. Generate Custom Messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DVL.msg"
  "msg/DVLBeam.msg"
  "msg/DVLDeadReckoning.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 3. Build C++ Nodes

# --- Publisher ---
add_executable(publisher src/publisher.cpp)
target_include_directories(publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(publisher
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
)

# Link external JSON library
target_link_libraries(publisher nlohmann_json::nlohmann_json)

# Link the messages generated in THIS package
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(publisher "${cpp_typesupport_target}")

# --- Subscriber ---
add_executable(subscriber src/subscriber.cpp)
target_include_directories(subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(subscriber
  rclcpp
  std_msgs
  geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target_sub
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(subscriber "${cpp_typesupport_target_sub}")

# Add node dvl_republisher
add_executable(dvl_republisher src/dvl_republisher.cpp)
target_include_directories(dvl_republisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(dvl_republisher
  rclcpp
  geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target_sub
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(dvl_republisher "${cpp_typesupport_target_sub}")

# 4. Install C++ Targets
install(TARGETS publisher subscriber dvl_republisher
  DESTINATION lib/${PROJECT_NAME}
)

# 5. Install Python Executable Scripts
install(PROGRAMS
  scripts/publisher.py
  scripts/subscriber.py
  scripts/subscriber_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

# 6. Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 7. Export
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

ament_package()