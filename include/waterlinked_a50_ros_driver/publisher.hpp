#ifndef DVL_A50_PUBLISHER_HPP
#define DVL_A50_PUBLISHER_HPP

#include <ros/ros.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <std_msgs/String.h>
#include <waterlinked_a50_ros_driver/DVL.h>
#include <waterlinked_a50_ros_driver/DVLBeam.h>

class DVLA50Publisher {
public:
    DVLA50Publisher();
    ~DVLA50Publisher();
    void run();

private:
    void connect();
    std::string getData();

    ros::NodeHandle nh_;
    ros::Publisher pub_raw_;
    ros::Publisher pub_;
    
    int sock_;
    std::string tcp_ip_;
    int tcp_port_;
    bool do_log_raw_data_;
    std::string old_json_;

    waterlinked_a50_ros_driver::DVL dvl_msg_;
    waterlinked_a50_ros_driver::DVLBeam beam0_;
    waterlinked_a50_ros_driver::DVLBeam beam1_;
    waterlinked_a50_ros_driver::DVLBeam beam2_;
    waterlinked_a50_ros_driver::DVLBeam beam3_;
};

#endif