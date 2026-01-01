#include "dvl_a50_ros_driver/dvl_republisher.hpp"

namespace dvl_a50_ros_driver

{

dvl_republisher::dvl_republisher() : Node("dvl_republisher")
{
  RCLCPP_INFO(this->get_logger(), "dvl_republisher has been started");

  // Declare and get parameters
  this->declare_parameter<std::string>("dvl_topic", "/dvl/velocity");
  this->declare_parameter<std::string>("output_twist_stamped_topic", "/dvl/twist_stamped");
  
  std::string input_topic = this->get_parameter("dvl_topic").as_string();
  std::string output_topic = this->get_parameter("output_twist_stamped_topic").as_string();

  // Create publisher for TwistWithCovarianceStamped messages
  dvl_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    output_topic, 10);

  // Subscribe to DVL velocity messages
  dvl_sub_ = this->create_subscription<dvl_a50_ros_driver::msg::DVL>(
    input_topic, 10,
    std::bind(&dvl_republisher::dvl_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Subscribed to %s, publishing to %s", 
              input_topic.c_str(), output_topic.c_str());
}

void dvl_republisher::dvl_callback(const dvl_a50_ros_driver::msg::DVL::SharedPtr msg)
{
  // Only publish if velocity is valid
  if (!msg->velocity_valid)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "DVL velocity not valid, skipping message");
    return;
  }

  // Create TwistWithCovarianceStamped message
  geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
  
  // Copy header information
  twist_msg.header = msg->header;
  
  // Set linear velocities (DVL measures linear velocity in body frame)
  twist_msg.twist.twist.linear.x = msg->velocity.x;
  twist_msg.twist.twist.linear.y = msg->velocity.y;
  twist_msg.twist.twist.linear.z = msg->velocity.z;
  
  // Angular velocities are not measured by DVL, set to zero
  twist_msg.twist.twist.angular.x = 0.0;
  twist_msg.twist.twist.angular.y = 0.0;
  twist_msg.twist.twist.angular.z = 0.0;
  
  // Populate covariance matrix (6x6 = 36 elements)
  // The covariance is stored as a row-major array:
  // [x, y, z, rot_x, rot_y, rot_z]
  // FOM (Figure of Merit) represents the standard deviation in m/s
  // Variance = (std_dev)^2
  double variance = msg->fom * msg->fom;
  
  // Initialize all covariances to zero
  std::fill(twist_msg.twist.covariance.begin(), twist_msg.twist.covariance.end(), 0.0);
  
  // Set linear velocity covariances (diagonal elements)
  twist_msg.twist.covariance[0] = variance;   // x variance
  twist_msg.twist.covariance[7] = variance;   // y variance
  twist_msg.twist.covariance[14] = variance;  // z variance
  
  // Angular velocity covariances are set to a large value since DVL doesn't measure them
  // This indicates high uncertainty
  twist_msg.twist.covariance[21] = 999999.0;  // rot_x variance (unknown)
  twist_msg.twist.covariance[28] = 999999.0;  // rot_y variance (unknown)
  twist_msg.twist.covariance[35] = 999999.0;  // rot_z variance (unknown)
  
  // Publish the message
  dvl_twist_pub_->publish(twist_msg);
}

}  // namespace namespace dvl_a50_ros_driver

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dvl_a50_ros_driver::dvl_republisher>());
  rclcpp::shutdown();
  return 0;
}