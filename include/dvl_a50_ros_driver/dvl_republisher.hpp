#ifndef DVL_A50_PUBLISHER_HPP
#define DVL_A50_PUBLISHER_HPP

#include "rclcpp/rclcpp.hpp"

#include "dvl_a50_ros_driver/msg/dvl.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

namespace dvl_a50_ros_driver
{

class dvl_republisher : public rclcpp::Node
{
public:
  dvl_republisher();

private:
  void dvl_callback(const dvl_a50_ros_driver::msg::DVL::SharedPtr msg);
  
  rclcpp::Subscription<dvl_a50_ros_driver::msg::DVL>::SharedPtr dvl_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_twist_pub_;
};

}  // namespace dvl_a50_ros_driver

#endif