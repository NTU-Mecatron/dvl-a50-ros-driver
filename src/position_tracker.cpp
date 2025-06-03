#include <ros/ros.h>
#include <dvl_a50_ros_driver/DVL.h>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class PositionTracker {
public:
    PositionTracker() : x_(0.0), y_(0.0), z_(0.0) {
        std::string log_dir = "logs";
        system(("mkdir -p " + log_dir).c_str());

        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << log_dir << "/position_log_";
        oss << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".json";
        log_file_ = oss.str();

        log_data_ = json::array();
        dvl_sub_ = nh_.subscribe("dvl/data", 10, &PositionTracker::dvlCallback, this);
        timer_ = nh_.createTimer(ros::Duration(30.0), &PositionTracker::logPosition, this);

        ROS_INFO("DVL posit tracker initialized. Logging to: %s", log_file_.c_str());
    }

    ~PositionTracker() {
        saveToFile();
    }

private:
    void dvlCallback(const dvl_a50_ros_driver::DVL::ConstPtr& msg) {
        double dt = 0.1;
        x_ += msg->velocity.x * dt;
        y_ += msg->velocity.y * dt;
        z_ += msg->velocity.z * dt;
    }

    void logPosition(const ros::TimerEvent&) {
        json entry;
        entry["timestamp"] = ros::Time::now().toSec();
        entry["position"] = {x_, y_, z_};

        log_data_.push_back(entry);

        saveToFile();
        ROS_INFO("Position logged: [%.2f, %.2f, %.2f]", x_, y_, z_);
    }

    void saveToFile() {
        std::ofstream file(log_file_);
        file << std::setw(4) << log_data_ << std::endl;
    }

    ros::NodeHandle nh_;
    ros::Subscriber dvl_sub_;
    ros::Timer timer_;

    double x_, y_, z_;
    std::string log_file_;
    json log_data_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "position_tracker");
    PositionTracker tracker;
    ros::spin();
    return 0;
}
