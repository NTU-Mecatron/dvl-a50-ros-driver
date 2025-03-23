#include <waterlinked_a50_ros_driver/subscriber.hpp>

DVLA50Subscriber::DVLA50Subscriber() {
    raw_sub_ = nh_.subscribe("dvl/json_data", 10, &DVLA50Subscriber::callbackRAW, this);
    dvl_sub_ = nh_.subscribe("dvl/data", 10, &DVLA50Subscriber::callback, this);
}

void DVLA50Subscriber::callbackRAW(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Data received: %s", msg->data.c_str());
}

void DVLA50Subscriber::callback(const waterlinked_a50_ros_driver::DVL::ConstPtr& msg) {
    ROS_INFO("Time received: %f", msg->time);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "a50_sub");
    DVLA50Subscriber subscriber;
    ros::spin();
    return 0;
}