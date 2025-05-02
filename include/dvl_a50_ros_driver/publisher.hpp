#ifndef DVL_A50_PUBLISHER_HPP
#define DVL_A50_PUBLISHER_HPP

#include <ros/ros.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

using namespace std;

class DVLA50Publisher {
public:
    DVLA50Publisher(ros::NodeHandle& nh);
    ~DVLA50Publisher();
    void run();

private:
    void connect();
    string getData();
    bool send_dvl_command(string cmd);
    bool reset_dead_reckoning(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool calibrate_gyro(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool get_config(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool turn_off(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool turn_on(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool toggle(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

    ros::NodeHandle nh_;
    ros::Publisher pub_raw_;
    ros::Publisher pub_velocity_;
    ros::Publisher pub_dead_reckoning_;
    ros::ServiceServer reset_dead_reckoning_server_;
    ros::ServiceServer calibrate_gyro_server_;
    ros::ServiceServer get_config_server_;
    ros::ServiceServer turn_off_server_;
    ros::ServiceServer turn_on_server_;
    ros::ServiceServer toggle_server;

    string reset_dead_reckoning_service, calibrate_gyro_service, get_config_service, turn_off_service, turn_on_service, toggle_service;
    
    int sock_;
    string tcp_ip_;
    int tcp_port_;
    bool do_log_raw_data_;
    string old_json_;
    string dvl_topic, dvl_raw_topic, dead_reckoning_topic;
};

#endif
