cmake_minimum_required(VERSION 3.8)
project(dvl_a50_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# 2. Build C++ Nodes

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

# Add node dvl_republisher
add_executable(dvl_republisher src/dvl_republisher.cpp)
target_include_directories(dvl_republisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(dvl_republisher
  rclcpp
  std_msgs
  geometry_msgs
)

# Link external JSON library for parsing raw DVL data
target_link_libraries(dvl_republisher nlohmann_json::nlohmann_json)

# 3. Install C++ Targets
install(TARGETS publisher dvl_republisher
  DESTINATION lib/${PROJECT_NAME}
)

# 4. Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 5. Export
ament_export_include_directories(include)

ament_package()