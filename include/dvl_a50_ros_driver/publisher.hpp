#ifndef DVL_A50_PUBLISHER_HPP
#define DVL_A50_PUBLISHER_HPP

#include <ros/ros.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <std_msgs/String.h>

using namespace std;

class DVLA50Publisher {
public:
    DVLA50Publisher(ros::NodeHandle& nh);
    ~DVLA50Publisher();
    void run();

private:
    void connect();
    string getData();

    ros::NodeHandle nh_;
    ros::Publisher pub_raw_;
    ros::Publisher pub_velocity_;
    ros::Publisher pub_dead_reckoning_;
    
    int sock_;
    string tcp_ip_;
    int tcp_port_;
    bool do_log_raw_data_;
    string old_json_;
};

#endif
