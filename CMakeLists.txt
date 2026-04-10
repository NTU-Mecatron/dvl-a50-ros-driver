cmake_minimum_required(VERSION 3.8)
project(dvl_a50_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# 2. Build C++ Nodes

# --- Publisher component library ---
add_library(publisher_component SHARED src/publisher.cpp)
target_include_directories(publisher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(publisher_component
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  geometry_msgs
)

# Link external JSON library
target_link_libraries(publisher_component nlohmann_json::nlohmann_json)
rclcpp_components_register_nodes(publisher_component "dvl::RawJsonPublisher")

# --- dvl_republisher component library ---
add_library(dvl_republisher_component SHARED src/dvl_republisher.cpp)
target_include_directories(dvl_republisher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(dvl_republisher_component
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
)

# Link external JSON library for parsing raw DVL data
target_link_libraries(dvl_republisher_component nlohmann_json::nlohmann_json)
rclcpp_components_register_nodes(dvl_republisher_component "dvl::TwistPublisher")

# 3. Install C++ Targets
install(TARGETS
  publisher_component
  dvl_republisher_component
  DESTINATION lib
)

# 4. Install launch files
install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

# 5. Export
ament_export_include_directories(include)
ament_export_dependencies(rclcpp_components)

ament_package()