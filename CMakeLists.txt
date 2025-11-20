cmake_minimum_required(VERSION 3.8)
project(dvl_a50_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
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
# We use the explicit target name matching the project name + rosidl suffix
rosidl_target_interfaces(publisher
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

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

rosidl_target_interfaces(subscriber
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 4. Install C++ Targets
install(TARGETS publisher subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# 5. Install Python Modules
ament_python_install_package(${PROJECT_NAME})

# 6. Install Python Executable Scripts
install(PROGRAMS
  scripts/publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# 7. Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 8. Export
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

ament_package()