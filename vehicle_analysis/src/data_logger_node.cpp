#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <filesystem>
#include <regex>
#include <set>

class DataLoggerNode : public rclcpp::Node
{
public:
    DataLoggerNode() : Node("data_logger_node"), mode_("unknown"), logging_enabled_(false)
    {
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&DataLoggerNode::cmd_vel_callback, this, std::placeholders::_1));
        
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&DataLoggerNode::odom_callback, this, std::placeholders::_1));
        
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&DataLoggerNode::scan_callback, this, std::placeholders::_1));
        
        racing_state_sub_ = create_subscription<std_msgs::msg::String>(
            "/racing/state", 10,
            std::bind(&DataLoggerNode::racing_state_callback, this, std::placeholders::_1));
        
        formation_state_sub_ = create_subscription<std_msgs::msg::String>(
            "/formation_lap/state", 10,
            std::bind(&DataLoggerNode::formation_state_callback, this, std::placeholders::_1));
        
        segment_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/current_segment", 10,
            std::bind(&DataLoggerNode::segment_callback, this, std::placeholders::_1));
        
        std::string pkg_share = ament_index_cpp::get_package_share_directory("vehicle_analysis");
        log_dir_ = pkg_share + "/data/";
        std::filesystem::create_directories(log_dir_);
        
        RCLCPP_INFO(get_logger(), "Data Logger Node started");
        RCLCPP_INFO(get_logger(), "Log directory: %s", log_dir_.c_str());
    }
    
    ~DataLoggerNode()
    {
        if (csv_file_.is_open()) {
            csv_file_.close();
            RCLCPP_INFO(get_logger(), "Log file closed: %s", current_filename_.c_str());
        }
    }

private:
    int find_next_session_id(const std::string& mode)
    {
        std::set<int> existing_ids;
        
        for (const auto& entry : std::filesystem::directory_iterator(log_dir_)) {
            if (entry.is_regular_file()) {
                std::string filename = entry.path().filename().string();
                
                std::regex pattern("(formation|racing)_(\\d+)\\.csv");
                std::smatch match;
                if (std::regex_match(filename, match, pattern)) {
                    int id = std::stoi(match[2]);
                    existing_ids.insert(id);
                }
            }
        }
        
        for (int id = 0; id < 1000; ++id) {
            std::string formation_file = log_dir_ + "formation_" + std::to_string(id) + ".csv";
            std::string racing_file = log_dir_ + "racing_" + std::to_string(id) + ".csv";
            
            bool formation_exists = std::filesystem::exists(formation_file);
            bool racing_exists = std::filesystem::exists(racing_file);
            
            if (mode == "formation") {
                if (!formation_exists) {
                    return id;
                }
            } else if (mode == "racing") {
                if (formation_exists && !racing_exists) {
                    return id;
                }
            }
        }
        
        return 0;
    }
    
    void racing_state_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data.find("RACING") != std::string::npos) {
            if (!logging_enabled_ || mode_ != "racing") {
                mode_ = "racing";
                start_logging();
            }
        } else if (msg->data == "FINISHED" || msg->data == "COMPLETE") {
            if (logging_enabled_ && mode_ == "racing") {
                stop_logging();
            }
        }
    }
    
    void formation_state_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data.find("FORMATION") != std::string::npos) {
            if (!logging_enabled_ || mode_ != "formation") {
                mode_ = "formation";
                start_logging();
            }
        } else if (msg->data == "COMPLETE" || msg->data == "FINISHED") {
            if (logging_enabled_ && mode_ == "formation") {
                stop_logging();
            }
        }
    }
    
    void start_logging()
    {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
        
        int session_id = find_next_session_id(mode_);
        current_filename_ = log_dir_ + mode_ + "_" + std::to_string(session_id) + ".csv";
        
        csv_file_.open(current_filename_);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open log file: %s", current_filename_.c_str());
            return;
        }
        
        csv_file_ << "timestamp,segment_id,pos_x,pos_y,yaw,vel_linear,vel_angular,"
                  << "left_dist,right_dist,front_dist,"
                  << "steering_cmd,speed_cmd\n";
        
        logging_enabled_ = true;
        RCLCPP_INFO(get_logger(), "Logging started: %s", current_filename_.c_str());
    }
    
    void stop_logging()
    {
        if (csv_file_.is_open()) {
            csv_file_.close();
            RCLCPP_INFO(get_logger(), "Logging stopped: %s", current_filename_.c_str());
            RCLCPP_INFO(get_logger(), "Total rows logged: %lu", row_count_);
        }
        logging_enabled_ = false;
        row_count_ = 0;
    }
    
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_linear_ = msg->linear.x;
        last_cmd_angular_ = msg->angular.z;
    }
    
    void segment_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        last_segment_id_ = msg->data;
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        last_pos_x_ = msg->pose.pose.position.x;
        last_pos_y_ = msg->pose.pose.position.y;
        
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        last_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        
        last_vel_linear_ = msg->twist.twist.linear.x;
        last_vel_angular_ = msg->twist.twist.angular.z;
        
        write_row();
    }
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int num_readings = msg->ranges.size();
        if (num_readings == 0) return;
        
        int center = num_readings / 2;
        
        int left_start = center + static_cast<int>((60.0 / 164.0) * num_readings);
        int left_end = num_readings - 1;
        double left_sum = 0.0;
        int left_count = 0;
        for (int i = left_start; i <= left_end && i < num_readings; ++i) {
            if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > msg->range_min) {
                left_sum += msg->ranges[i];
                left_count++;
            }
        }
        last_left_dist_ = (left_count > 0) ? (left_sum / left_count) : 10.0;
        
        int right_start = 0;
        int right_end = center - static_cast<int>((60.0 / 164.0) * num_readings);
        double right_sum = 0.0;
        int right_count = 0;
        for (int i = right_start; i <= right_end && i < num_readings; ++i) {
            if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > msg->range_min) {
                right_sum += msg->ranges[i];
                right_count++;
            }
        }
        last_right_dist_ = (right_count > 0) ? (right_sum / right_count) : 10.0;
        
        int front_angle_samples = static_cast<int>((10.0 / 164.0) * num_readings);
        double front_sum = 0.0;
        int front_count = 0;
        for (int i = center - front_angle_samples; i < center + front_angle_samples; ++i) {
            if (i >= 0 && i < num_readings && std::isfinite(msg->ranges[i]) && msg->ranges[i] > msg->range_min) {
                front_sum += msg->ranges[i];
                front_count++;
            }
        }
        last_front_dist_ = (front_count > 0) ? (front_sum / front_count) : 10.0;
    }
    
    void write_row()
    {
        if (!logging_enabled_ || !csv_file_.is_open()) return;
        
        auto now = this->now();
        double timestamp = now.seconds();
        
        csv_file_ << std::fixed << std::setprecision(3)
                  << timestamp << ","
                  << last_segment_id_ << ","
                  << last_pos_x_ << ","
                  << last_pos_y_ << ","
                  << last_yaw_ << ","
                  << last_vel_linear_ << ","
                  << last_vel_angular_ << ","
                  << last_left_dist_ << ","
                  << last_right_dist_ << ","
                  << last_front_dist_ << ","
                  << last_cmd_angular_ << ","
                  << last_cmd_linear_ << "\n";
        
        row_count_++;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr racing_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr formation_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr segment_sub_;
    
    std::ofstream csv_file_;
    std::string log_dir_;
    std::string current_filename_;
    std::string mode_;
    bool logging_enabled_;
    size_t row_count_ = 0;
    
    int last_segment_id_ = -1;
    double last_pos_x_ = 0.0;
    double last_pos_y_ = 0.0;
    double last_yaw_ = 0.0;
    double last_vel_linear_ = 0.0;
    double last_vel_angular_ = 0.0;
    double last_cmd_linear_ = 0.0;
    double last_cmd_angular_ = 0.0;
    double last_left_dist_ = 10.0;
    double last_right_dist_ = 10.0;
    double last_front_dist_ = 10.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataLoggerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
