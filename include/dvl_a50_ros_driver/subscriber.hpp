#ifndef DVL_A50_SUBSCRIBER_HPP
#define DVL_A50_SUBSCRIBER_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <dvl_a50_ros_driver/DVL.h>
#include <dvl_a50_ros_driver/DVLBeam.h>

class DVLA50Subscriber {
public:
    DVLA50Subscriber();
    void callbackRAW(const std_msgs::String::ConstPtr& msg);
    void callback(const dvl_a50_ros_driver::DVL::ConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber raw_sub_;
    ros::Subscriber dvl_sub_;
};

#endif
