#include "dvl_a50_ros_driver/dvl_republisher.hpp"

namespace dvl_a50_ros_driver

{

dvl_republisher::dvl_republisher() : Node("dvl_republisher")
{
    RCLCPP_INFO(this->get_logger(), "dvl_republisher has been started");

    // Declare and get parameters
    this->declare_parameter<std::string>("dvl_topic", "dvl/original_data");
    this->declare_parameter<std::string>("output_twist_stamped_topic", "dvl/twist_stamped");
    this->declare_parameter<bool>("use_original_covariance", true);
    this->declare_parameter<bool>("use_fom_to_compute_covariance", false);
    this->declare_parameter<double>("linear_vel_var_x", 0.01);
    this->declare_parameter<double>("linear_vel_var_y", 0.01);
    this->declare_parameter<double>("linear_vel_var_z", 0.01);

    std::string input_topic = this->get_parameter("dvl_topic").as_string();
    std::string output_topic = this->get_parameter("output_twist_stamped_topic").as_string();
    use_original_covariance_ = this->get_parameter("use_original_covariance").as_bool();
    use_fom_to_compute_covariance_ = this->get_parameter("use_fom_to_compute_covariance").as_bool();
    linear_vel_var_x_ = this->get_parameter("linear_vel_var_x").as_double();
    linear_vel_var_y_ = this->get_parameter("linear_vel_var_y").as_double();
    linear_vel_var_z_ = this->get_parameter("linear_vel_var_z").as_double();

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
    // Convert from FRD to FLU by negating Y and Z
    twist_msg.twist.twist.linear.x = msg->velocity.x;
    twist_msg.twist.twist.linear.y = -msg->velocity.y;
    twist_msg.twist.twist.linear.z = -msg->velocity.z;

    // Angular velocities are not measured by DVL, set to zero
    twist_msg.twist.twist.angular.x = 0.0;
    twist_msg.twist.twist.angular.y = 0.0;
    twist_msg.twist.twist.angular.z = 0.0;

    // Populate covariance matrix (6x6 = 36 elements)
    // The covariance is stored as a row-major array:
    // [x, y, z, rot_x, rot_y, rot_z]
    double variance_x, variance_y, variance_z;
    
    // Priority order: 1) DVL covariance, 2) FOM, 3) User-defined parameters
    if (use_original_covariance_ && msg->covariance.size() == 9)
    {
        // Extract diagonal elements from DVL covariance matrix (3x3 row-major)
        // covariance[0] = xx, covariance[4] = yy, covariance[8] = zz
        variance_x = msg->covariance[0];  // xx
        variance_y = msg->covariance[4];  // yy
        variance_z = msg->covariance[8];  // zz
    }
    else if (use_fom_to_compute_covariance_)
    {
        // FOM (Figure of Merit) represents the standard deviation in m/s
        // Variance = (std_dev)^2
        variance_x = variance_y = variance_z = msg->fom * msg->fom;
    }
    else
    {
        // Use user-defined covariances from parameters
        variance_x = linear_vel_var_x_;
        variance_y = linear_vel_var_y_;
        variance_z = linear_vel_var_z_;
    }

    // Initialize all covariances to zero
    std::fill(twist_msg.twist.covariance.begin(), twist_msg.twist.covariance.end(), 0.0);

    // Set linear velocity covariances (diagonal elements)
    twist_msg.twist.covariance[0] = variance_x;  // x variance
    twist_msg.twist.covariance[7] = variance_y;  // y variance
    twist_msg.twist.covariance[14] = variance_z; // z variance

    // Angular velocity covariances are set to a large value since DVL doesn't measure them
    // This indicates high uncertainty
    twist_msg.twist.covariance[21] = 999999.0; // rot_x variance (unknown)
    twist_msg.twist.covariance[28] = 999999.0; // rot_y variance (unknown)
    twist_msg.twist.covariance[35] = 999999.0; // rot_z variance (unknown)

    // Publish the message
    dvl_twist_pub_->publish(twist_msg);
}

} // namespace dvl_a50_ros_driver

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dvl_a50_ros_driver::dvl_republisher>());
    rclcpp::shutdown();
    return 0;
}