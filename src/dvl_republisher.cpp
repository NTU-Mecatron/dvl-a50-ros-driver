#include "dvl_a50_ros_driver/dvl_republisher.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace dvl_a50_ros_driver
{

dvl_republisher::dvl_republisher() : Node("dvl_republisher")
{
    RCLCPP_INFO(this->get_logger(), "dvl_republisher has been started");

    // Declare and get parameters
    this->declare_parameter<std::string>("dvl_raw_topic", "dvl/raw_data");
    this->declare_parameter<std::string>("output_twist_stamped_topic", "dvl/twist_stamped");
    this->declare_parameter<std::string>("dvl_frame_id", "dvl_link");
    this->declare_parameter<bool>("use_original_covariance", true);
    this->declare_parameter<bool>("use_fom_to_compute_covariance", false);
    this->declare_parameter<double>("linear_vel_var_x", 0.01);
    this->declare_parameter<double>("linear_vel_var_y", 0.01);
    this->declare_parameter<double>("linear_vel_var_z", 0.01);

    std::string raw_input_topic = this->get_parameter("dvl_raw_topic").as_string();
    std::string twist_output_topic = this->get_parameter("output_twist_stamped_topic").as_string();
    dvl_frame_id_ = this->get_parameter("dvl_frame_id").as_string();
    use_original_covariance_ = this->get_parameter("use_original_covariance").as_bool();
    use_fom_to_compute_covariance_ = this->get_parameter("use_fom_to_compute_covariance").as_bool();
    linear_vel_var_x_ = this->get_parameter("linear_vel_var_x").as_double();
    linear_vel_var_y_ = this->get_parameter("linear_vel_var_y").as_double();
    linear_vel_var_z_ = this->get_parameter("linear_vel_var_z").as_double();

    // Create publisher
    dvl_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        twist_output_topic, 10);

    // Subscribe to raw DVL JSON messages
    dvl_raw_sub_ = this->create_subscription<std_msgs::msg::String>(
        raw_input_topic, 10,
        std::bind(&dvl_republisher::raw_dvl_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s, publishing twist to %s",
                raw_input_topic.c_str(), twist_output_topic.c_str());
}

void dvl_republisher::raw_dvl_callback(const std_msgs::msg::String::SharedPtr msg)
{
    try
    {
        json data = json::parse(msg->data);

        // Only process velocity messages
        if (data["type"] != "velocity")
        {
            return;
        }

        // Check if velocity is valid
        bool velocity_valid = data["velocity_valid"];
        if (!velocity_valid)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "DVL velocity not valid, skipping twist message");
            return;
        }

        // Create TwistWithCovarianceStamped message
        geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = dvl_frame_id_;

        // Set linear velocities (DVL measures linear velocity in body frame)
        // Convert from FRD to FLU by negating Y and Z
        twist_msg.twist.twist.linear.x = data["vx"];
        twist_msg.twist.twist.linear.y = -static_cast<double>(data["vy"]);
        twist_msg.twist.twist.linear.z = -static_cast<double>(data["vz"]);

        // Angular velocities are not measured by DVL, set to zero
        twist_msg.twist.twist.angular.x = 0.0;
        twist_msg.twist.twist.angular.y = 0.0;
        twist_msg.twist.twist.angular.z = 0.0;

        // Populate covariance matrix (6x6 = 36 elements)
        // The covariance is stored as a row-major array:
        // [x, y, z, rot_x, rot_y, rot_z]
        double variance_x, variance_y, variance_z;

        // Priority order: 1) DVL covariance, 2) FOM, 3) User-defined parameters
        if (use_original_covariance_ && data.contains("covariance") && 
            data["covariance"].is_array() && data["covariance"].size() == 3)
        {
            // Extract diagonal elements from DVL covariance matrix (3x3)
            // covariance[0][0] = xx, covariance[1][1] = yy, covariance[2][2] = zz
            variance_x = data["covariance"][0][0];
            variance_y = data["covariance"][1][1];
            variance_z = data["covariance"][2][2];
        }
        else if (use_fom_to_compute_covariance_)
        {
            // FOM (Figure of Merit) represents the standard deviation in m/s
            // Variance = (std_dev)^2
            double fom = data["fom"];
            variance_x = variance_y = variance_z = fom * fom;
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
        twist_msg.twist.covariance[0] = variance_x;   // x variance
        twist_msg.twist.covariance[7] = variance_y;   // y variance
        twist_msg.twist.covariance[14] = variance_z;  // z variance

        // Angular velocity covariances are set to a large value since DVL doesn't measure them
        // This indicates high uncertainty
        twist_msg.twist.covariance[21] = 999999.0;  // rot_x variance (unknown)
        twist_msg.twist.covariance[28] = 999999.0;  // rot_y variance (unknown)
        twist_msg.twist.covariance[35] = 999999.0;  // rot_z variance (unknown)

        // Publish the message
        dvl_twist_pub_->publish(twist_msg);
    }
    catch (const json::parse_error &e)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Failed to parse DVL JSON: %s", e.what());
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Error processing DVL data: %s", e.what());
    }
}

}  // namespace dvl_a50_ros_driver

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dvl_a50_ros_driver::dvl_republisher>());
    rclcpp::shutdown();
    return 0;
}