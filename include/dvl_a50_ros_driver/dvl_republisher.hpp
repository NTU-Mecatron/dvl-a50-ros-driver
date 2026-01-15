#ifndef DVL_A50_REPUBLISHER_HPP
#define DVL_A50_REPUBLISHER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

namespace dvl_a50_ros_driver
{

  class dvl_republisher : public rclcpp::Node
  {
  public:
    dvl_republisher();

  private:
    void raw_dvl_callback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dvl_raw_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_twist_pub_;

    std::string dvl_frame_id_;
    bool use_original_covariance_;
    bool use_fom_to_compute_covariance_;
    double linear_vel_var_x_;
    double linear_vel_var_y_;
    double linear_vel_var_z_;
  };

} // namespace dvl_a50_ros_driver

#endif