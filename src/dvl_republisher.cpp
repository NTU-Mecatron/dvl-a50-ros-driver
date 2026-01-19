#include "dvl_a50_ros_driver/dvl_republisher.hpp"

using json = nlohmann::json;
using TwistWithCovarianceStamped = geometry_msgs::msg::TwistWithCovarianceStamped;

namespace dvl_a50_ros_driver
{

dvl_republisher::dvl_republisher() : Node("dvl_republisher")
{
    RCLCPP_INFO(this->get_logger(), "dvl_republisher has been started");
    
    load_parameters();
    setup_pub_sub();
    initialize_twist_template();
}

void dvl_republisher::load_parameters()
{
    // Declare parameters with defaults
    this->declare_parameter<std::string>("dvl_raw_topic", "dvl/raw_data");
    this->declare_parameter<std::string>("output_twist_stamped_topic", "dvl/twist_stamped");
    this->declare_parameter<std::string>("dvl_frame_id", "dvl_link");
    this->declare_parameter<bool>("use_original_covariance", true);
    this->declare_parameter<bool>("use_fom_to_compute_covariance", false);
    this->declare_parameter<double>("linear_vel_var_x", 0.01);
    this->declare_parameter<double>("linear_vel_var_y", 0.01);
    this->declare_parameter<double>("linear_vel_var_z", 0.01);

    // Get parameter values
    dvl_frame_id_ = this->get_parameter("dvl_frame_id").as_string();
    use_original_covariance_ = this->get_parameter("use_original_covariance").as_bool();
    use_fom_to_compute_covariance_ = this->get_parameter("use_fom_to_compute_covariance").as_bool();
    linear_vel_var_x_ = this->get_parameter("linear_vel_var_x").as_double();
    linear_vel_var_y_ = this->get_parameter("linear_vel_var_y").as_double();
    linear_vel_var_z_ = this->get_parameter("linear_vel_var_z").as_double();
}

void dvl_republisher::setup_pub_sub()
{
    const auto raw_input_topic =
        this->get_parameter("dvl_raw_topic").as_string();

    const auto twist_output_topic =
        this->get_parameter("output_twist_stamped_topic").as_string();

    dvl_twist_pub_ = this->create_publisher<TwistWithCovarianceStamped>(
        twist_output_topic, 10);
    
    dvl_raw_sub_ = this->create_subscription<std_msgs::msg::String>(
        raw_input_topic, 10,
        std::bind(&dvl_republisher::raw_dvl_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Subscribed to %s, publishing twist to %s",
                raw_input_topic.c_str(), twist_output_topic.c_str());
}

void dvl_republisher::initialize_twist_template()
{
    twist_template_.header.frame_id = dvl_frame_id_;
    
    // Angular velocities (DVL doesn't measure rotation)
    twist_template_.twist.twist.angular.x = 0.0;
    twist_template_.twist.twist.angular.y = 0.0;
    twist_template_.twist.twist.angular.z = 0.0;
    
    // Initialize covariance matrix
    std::fill(twist_template_.twist.covariance.begin(), twist_template_.twist.covariance.end(), 0.0);
    
    // Angular velocity covariances (high uncertainty - not measured)
    constexpr double UNKNOWN_ANGULAR_VARIANCE = 999999.0;
    twist_template_.twist.covariance[21] = UNKNOWN_ANGULAR_VARIANCE;  // rot_x
    twist_template_.twist.covariance[28] = UNKNOWN_ANGULAR_VARIANCE;  // rot_y
    twist_template_.twist.covariance[35] = UNKNOWN_ANGULAR_VARIANCE;  // rot_z
}

void dvl_republisher::raw_dvl_callback(const std_msgs::msg::String::SharedPtr msg)
{
    try
    {
        json data = json::parse(msg->data);

        if (data["type"] != "velocity" || !is_velocity_valid(data))
        {
            return;
        }

        TwistWithCovarianceStamped twist_msg = twist_template_;
        twist_msg.header.stamp = this->now();

        populate_linear_velocities(twist_msg, data);
        
        double variance_x, variance_y, variance_z;
        compute_velocity_covariances(variance_x, variance_y, variance_z, data);
        
        twist_msg.twist.covariance[0] = variance_x;
        twist_msg.twist.covariance[7] = variance_y;
        twist_msg.twist.covariance[14] = variance_z;

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

bool dvl_republisher::is_velocity_valid(const json& data)
{
    bool velocity_valid = data["velocity_valid"];
    if (!velocity_valid)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "DVL velocity not valid, skipping twist message");
    }
    return velocity_valid;
}

void dvl_republisher::populate_linear_velocities(TwistWithCovarianceStamped& twist_msg,
                                                 const json& data) const
{
    // DVL measures linear velocity in body frame - convert from FRD to FLU
    twist_msg.twist.twist.linear.x = data["vx"];
    twist_msg.twist.twist.linear.y = -static_cast<double>(data["vy"]);
    twist_msg.twist.twist.linear.z = -static_cast<double>(data["vz"]);
}

void dvl_republisher::compute_velocity_covariances(double& variance_x, double& variance_y,
                                                   double& variance_z, const json& data) const
{
    // Priority: 1) DVL covariance matrix, 2) FOM, 3) User parameters
    if (use_original_covariance_ && data.contains("covariance") &&
        data["covariance"].is_array() && data["covariance"].size() == 3)
    {
        // Extract diagonal from DVL's 3x3 covariance matrix
        variance_x = data["covariance"][0][0];
        variance_y = data["covariance"][1][1];
        variance_z = data["covariance"][2][2];
    }
    else if (use_fom_to_compute_covariance_)
    {
        // FOM = standard deviation (m/s), variance = σ²
        const double fom = data["fom"];
        variance_x = variance_y = variance_z = fom * fom;
    }
    else
    {
        // Use configured parameter values
        variance_x = linear_vel_var_x_;
        variance_y = linear_vel_var_y_;
        variance_z = linear_vel_var_z_;
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