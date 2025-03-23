#include <waterlinked_a50_ros_driver/publisher.hpp>
#include <nlohmann/json.hpp>
#include <iostream>

using json = nlohmann::json;

DVLA50Publisher::DVLA50Publisher() : sock_(-1) {
    tcp_ip_ = ros::param::param<std::string>("~ip", "10.42.0.186");
    tcp_port_ = ros::param::param<int>("~port", 16171);
    do_log_raw_data_ = ros::param::param<bool>("~do_log_raw_data", false);

    pub_raw_ = nh_.advertise<std_msgs::String>("dvl/json_data", 10);
    pub_ = nh_.advertise<waterlinked_a50_ros_driver::DVL>("dvl/data", 10);

    connect();
}

DVLA50Publisher::~DVLA50Publisher() {
    if (sock_ >= 0) {
        close(sock_);
    }
}

void DVLA50Publisher::connect() {
    if (sock_ >= 0) {
        close(sock_);
    }

    sock_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ < 0) {
        ROS_ERROR("Socket creation error");
        ros::Duration(1.0).sleep();
        connect();
        return;
    }

    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(tcp_port_);

    if (inet_pton(AF_INET, tcp_ip_.c_str(), &serv_addr.sin_addr) <= 0) {
        ROS_ERROR("Invalid address");
        ros::Duration(1.0).sleep();
        connect();
        return;
    }

    if (::connect(sock_, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        ROS_ERROR("Connection failed");
        ros::Duration(1.0).sleep();
        connect();
        return;
    }

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
}

std::string DVLA50Publisher::getData() {
    std::string raw_data;
    std::vector<char> buffer(1024);
    
    while (raw_data.find('\n') == std::string::npos) {
        ssize_t n = recv(sock_, buffer.data(), buffer.size(), 0);
        if (n < 1) {
            ROS_ERROR("Connection lost, reconnecting...");
            connect();
            continue;
        }
        raw_data.append(buffer.data(), n);
    }

    raw_data = old_json_ + raw_data;
    old_json_ = "";

    size_t pos = raw_data.find('\n');

    // check if we received the full json string
    if (pos != std::string::npos) {
        old_json_ = raw_data.substr(pos + 1);
        raw_data = raw_data.substr(0, pos);
    }

    return raw_data;
}

void DVLA50Publisher::run() {
    ros::Rate rate(10);

    while (ros::ok()) {
        std::string raw_data = getData();
        json data = json::parse(raw_data);

        std_msgs::String raw_msg;
        raw_msg.data = raw_data;

        if (do_log_raw_data_) {
            ROS_INFO("%s", raw_data.c_str());
            pub_raw_.publish(raw_msg);
            if (data["type"] != "velocity") {
                continue;
            }
        } else {
            if (data["type"] != "velocity") {
                continue;
            }
            pub_raw_.publish(raw_msg);
        }

        dvl_msg_.header.stamp = ros::Time::now();
        dvl_msg_.header.frame_id = "dvl_link";
        dvl_msg_.time = data["time"];
        dvl_msg_.velocity.x = data["vx"];
        dvl_msg_.velocity.y = data["vy"];
        dvl_msg_.velocity.z = data["vz"];
        dvl_msg_.fom = data["fom"];
        dvl_msg_.altitude = data["altitude"];
        dvl_msg_.velocity_valid = data["velocity_valid"];
        dvl_msg_.status = data["status"];
        dvl_msg_.form = data["format"];

        for (int i = 0; i < 4; ++i) {
            auto& beam = (i == 0) ? beam0_ : (i == 1) ? beam1_ : (i == 2) ? beam2_ : beam3_;
            const auto& trans = data["transducers"][i];
            
            beam.id = trans["id"];
            beam.velocity = trans["velocity"];
            beam.distance = trans["distance"];
            beam.rssi = trans["rssi"];
            beam.nsd = trans["nsd"];
            beam.valid = trans["beam_valid"];
        }

        dvl_msg_.beams = {beam0_, beam1_, beam2_, beam3_};
        pub_.publish(dvl_msg_);

        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "a50_pub");
    DVLA50Publisher publisher;
    
    try {
        publisher.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }

    return 0;
}
