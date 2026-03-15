#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "dvl_a50_ros_driver/dvl_republisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dvl_republisher>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
