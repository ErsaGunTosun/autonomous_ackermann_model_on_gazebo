#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <set>
#include <cmath>
#include <regex>
#include <iomanip>
#include <algorithm>

struct Segment {
    int id;
    int start_aruco_id;
    int end_aruco_id;
    std::string type;
    double base_speed;
    double learned_speed;
    double steering_multiplier;
    int success_count;
    int crash_count;
};

class SegmentRacingNode : public rclcpp::Node
{
public:
    enum class State {
        WAITING_START,
        RACING,
        LAP_COMPLETE,
        FINISHED
    };

    SegmentRacingNode() : Node("segment_racing_node")
    {
        this->declare_parameter<int>("total_laps", 3);
        this->declare_parameter<double>("steering_kp", 0.8);
        this->declare_parameter<double>("steering_kd", 0.1);
        this->declare_parameter<double>("min_speed", 0.15);
        this->declare_parameter<double>("max_speed", 0.6);
        this->declare_parameter<double>("default_speed", 0.35);
        this->declare_parameter<double>("speed_increase_rate", 1.10);
        this->declare_parameter<double>("speed_decrease_rate", 0.80);
        this->declare_parameter<double>("crash_threshold", 0.15);
        this->declare_parameter<double>("start_box_radius", 1.5);
        this->declare_parameter<double>("min_leave_distance", 3.0);
        
        total_laps_ = this->get_parameter("total_laps").as_int();
        steering_kp_ = this->get_parameter("steering_kp").as_double();
        steering_kd_ = this->get_parameter("steering_kd").as_double();
        min_speed_ = this->get_parameter("min_speed").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        default_speed_ = this->get_parameter("default_speed").as_double();
        speed_increase_rate_ = this->get_parameter("speed_increase_rate").as_double();
        speed_decrease_rate_ = this->get_parameter("speed_decrease_rate").as_double();
        crash_threshold_ = this->get_parameter("crash_threshold").as_double();
        start_box_radius_ = this->get_parameter("start_box_radius").as_double();
        min_leave_distance_ = this->get_parameter("min_leave_distance").as_double();
        
        current_state_ = State::WAITING_START;
        current_lap_ = 0;
        current_segment_id_ = -1;
        target_speed_ = default_speed_;
        
        left_distance_ = 10.0;
        right_distance_ = 10.0;
        front_distance_ = 10.0;
        min_distance_ = 10.0;
        previous_error_ = 0.0;
        
        start_recorded_ = false;
        has_left_start_ = false;
        had_crash_this_segment_ = false;
        
        segment_derivative_sum_ = 0.0;
        segment_control_count_ = 0;
        
        // TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Publishers
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        state_pub_ = create_publisher<std_msgs::msg::String>("/racing/state", 10);
        
        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(&SegmentRacingNode::lidar_callback, this, std::placeholders::_1));
        
        aruco_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "/aruco/detected_ids",
            10,
            std::bind(&SegmentRacingNode::aruco_callback, this, std::placeholders::_1));
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SegmentRacingNode::control_loop, this));
        load_segment_map();
        
        RCLCPP_INFO(get_logger(), "===========================================");
        RCLCPP_INFO(get_logger(), "   SEGMENT RACING NODE STARTED");
        RCLCPP_INFO(get_logger(), "   Total laps: %d", total_laps_);
        RCLCPP_INFO(get_logger(), "   Segments loaded: %zu", segments_.size());
        RCLCPP_INFO(get_logger(), "===========================================");
    }

private:
    State current_state_;
    int current_lap_;
    int total_laps_;
    int current_segment_id_;
    double target_speed_;
    
    double start_x_, start_y_;
    double current_x_, current_y_;
    bool start_recorded_;
    bool has_left_start_;
    
    double left_distance_;
    double right_distance_;
    double front_distance_;
    double min_distance_;
    double previous_error_;
    
    double steering_kp_;
    double steering_kd_;
    double min_speed_;
    double max_speed_;
    double default_speed_;
    double speed_increase_rate_;
    double speed_decrease_rate_;
    double crash_threshold_;
    double start_box_radius_;
    double min_leave_distance_;
    
    std::vector<Segment> segments_;
    std::map<int, int> aruco_to_segment_;
    std::set<int> detected_markers_this_lap_;
    bool had_crash_this_segment_;
    
    double segment_derivative_sum_;
    int segment_control_count_;
    
    std::vector<double> lap_times_;
    rclcpp::Time lap_start_time_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr aruco_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    void load_segment_map() {
        std::string nav_pkg_path = ament_index_cpp::get_package_share_directory("vehicle_navigation");
        std::string segments_path = nav_pkg_path + "/maps/segment_map.yaml";
        
        std::ifstream file(segments_path);
        if (!file.is_open()) {
            RCLCPP_WARN(get_logger(), "No segment map found: %s", segments_path.c_str());
            RCLCPP_WARN(get_logger(), "Using default speed: %.2f m/s", default_speed_);
            return;
        }
        
        segments_.clear();
        aruco_to_segment_.clear();
        
        std::string line;
        Segment current_seg;
        bool in_segment = false;
        
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            
            if (line.find("- id:") != std::string::npos) {
                if (in_segment) {
                    segments_.push_back(current_seg);
                    aruco_to_segment_[current_seg.start_aruco_id] = segments_.size() - 1;
                }
                in_segment = true;
                current_seg = Segment();
                current_seg.steering_multiplier = 1.0;  // Default if not in YAML
                std::regex re("id:\\s*(\\d+)");
                std::smatch match;
                if (std::regex_search(line, match, re)) {
                    current_seg.id = std::stoi(match[1]);
                }
            }
            else if (line.find("start_aruco_id:") != std::string::npos && in_segment) {
                std::regex re("start_aruco_id:\\s*(\\d+)");
                std::smatch match;
                if (std::regex_search(line, match, re)) {
                    current_seg.start_aruco_id = std::stoi(match[1]);
                }
            }
            else if (line.find("end_aruco_id:") != std::string::npos && in_segment) {
                std::regex re("end_aruco_id:\\s*(\\d+)");
                std::smatch match;
                if (std::regex_search(line, match, re)) {
                    current_seg.end_aruco_id = std::stoi(match[1]);
                }
            }
            else if (line.find("type:") != std::string::npos && in_segment) {
                std::regex re("type:\\s*(\\w+)");
                std::smatch match;
                if (std::regex_search(line, match, re)) {
                    current_seg.type = match[1];
                }
            }
            else if (line.find("learned_speed:") != std::string::npos && in_segment) {
                std::regex re("learned_speed:\\s*([\\d.]+)");
                std::smatch match;
                if (std::regex_search(line, match, re)) {
                    current_seg.learned_speed = std::stod(match[1]);
                }
            }
            else if (line.find("steering_multiplier:") != std::string::npos && in_segment) {
                std::regex re("steering_multiplier:\\s*([\\d.]+)");
                std::smatch match;
                if (std::regex_search(line, match, re)) {
                    current_seg.steering_multiplier = std::stod(match[1]);
                }
            }
            else if (line.find("base_speed:") != std::string::npos && in_segment) {
                std::regex re("base_speed:\\s*([\\d.]+)");
                std::smatch match;
                if (std::regex_search(line, match, re)) {
                    current_seg.base_speed = std::stod(match[1]);
                }
            }
            else if (line.find("success_count:") != std::string::npos && in_segment) {
                std::regex re("success_count:\\s*(\\d+)");
                std::smatch match;
                if (std::regex_search(line, match, re)) {
                    current_seg.success_count = std::stoi(match[1]);
                }
            }
            else if (line.find("crash_count:") != std::string::npos && in_segment) {
                std::regex re("crash_count:\\s*(\\d+)");
                std::smatch match;
                if (std::regex_search(line, match, re)) {
                    current_seg.crash_count = std::stoi(match[1]);
                }
            }
        }
        
        // Son segment'i ekle
        if (in_segment) {
            segments_.push_back(current_seg);
            aruco_to_segment_[current_seg.start_aruco_id] = segments_.size() - 1;
        }
        
        file.close();
        
        for (const auto& seg : segments_) {
            RCLCPP_INFO(get_logger(), "Segment %d: ArUco %d→%d, Type=%s, Speed=%.2f",
                        seg.id, seg.start_aruco_id, seg.end_aruco_id, seg.type.c_str(), seg.learned_speed);
        }
    }
    
    void save_segment_map() {
        std::string nav_pkg_path = ament_index_cpp::get_package_share_directory("vehicle_navigation");
        std::string segments_path = nav_pkg_path + "/maps/segment_map.yaml";
        
        std::ofstream file(segments_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to save segment map!");
            return;
        }
        
        file << "# Segment map - Updated after racing\n";
        file << "# Segments represent area BETWEEN ArUco markers\n";
        file << "# Total segments: " << segments_.size() << "\n\n";
        file << "segments:\n";
        
        for (const auto& seg : segments_) {
            file << "  - id: " << seg.id << "\n";
            file << "    start_aruco_id: " << seg.start_aruco_id << "\n";
            file << "    end_aruco_id: " << seg.end_aruco_id << "\n";
            file << "    type: " << seg.type << "\n";
            file << "    base_speed: " << std::fixed << std::setprecision(2) << seg.base_speed << "\n";
            file << "    learned_speed: " << std::fixed << std::setprecision(2) << seg.learned_speed << "\n";
            file << "    steering_multiplier: " << std::fixed << std::setprecision(2) << seg.steering_multiplier << "\n";
            file << "    success_count: " << seg.success_count << "\n";
            file << "    crash_count: " << seg.crash_count << "\n";
        }
        
        file.close();
        RCLCPP_INFO(get_logger(), "Segment map saved with updated speeds");
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        int num_readings = msg->ranges.size();
        if (num_readings == 0) return;
        
        int center = num_readings / 2;
        
        int front_angle_samples = static_cast<int>((10.0 / 164.0) * num_readings);
        double front_sum = 0.0;
        int front_count = 0;
        for (int i = center - front_angle_samples; i < center + front_angle_samples; ++i) {
            if (i >= 0 && i < num_readings && std::isfinite(msg->ranges[i]) && 
                msg->ranges[i] > msg->range_min && msg->ranges[i] < msg->range_max) {
                front_sum += msg->ranges[i];
                front_count++;
            }
        }
        front_distance_ = (front_count > 0) ? (front_sum / front_count) : 10.0;
        
        int left_start = center + static_cast<int>((60.0 / 164.0) * num_readings);  
        int left_end = num_readings - 1; 
        double left_sum = 0.0;
        int left_count = 0;
        for (int i = left_start; i <= left_end && i < num_readings; ++i) {
            if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > msg->range_min && msg->ranges[i] < msg->range_max) {
                left_sum += msg->ranges[i];
                left_count++;
            }
        }
        left_distance_ = (left_count > 0) ? (left_sum / left_count) : 10.0;
        
        int right_start = 0; 
        int right_end = center - static_cast<int>((60.0 / 164.0) * num_readings);
        double right_sum = 0.0;
        int right_count = 0;
        for (int i = right_start; i <= right_end && i >= 0; ++i) {
            if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > msg->range_min && msg->ranges[i] < msg->range_max) {
                right_sum += msg->ranges[i];
                right_count++;
            }
        }
        right_distance_ = (right_count > 0) ? (right_sum / right_count) : 10.0;
    
        min_distance_ = 10.0;
        for (int i = 0; i < num_readings; ++i) {
            if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > msg->range_min) {
                min_distance_ = std::min(min_distance_, static_cast<double>(msg->ranges[i]));
            }
        }
    }
    
    void aruco_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (current_state_ != State::RACING) return;
        
        for (int marker_id : msg->data) {
            if (detected_markers_this_lap_.find(marker_id) == detected_markers_this_lap_.end()) {
                detected_markers_this_lap_.insert(marker_id);
                
                auto it = aruco_to_segment_.find(marker_id);
                if (it != aruco_to_segment_.end()) {
                    if (current_segment_id_ >= 0 && current_segment_id_ < static_cast<int>(segments_.size())) {
                        update_segment_stats(current_segment_id_, !had_crash_this_segment_);
                    }
                    
                    current_segment_id_ = it->second;
                    target_speed_ = segments_[current_segment_id_].learned_speed;
                    had_crash_this_segment_ = false;
                    
                    segment_derivative_sum_ = 0.0;
                    segment_control_count_ = 0;
                    
                    RCLCPP_INFO(get_logger(), "SEGMENT %d: ArUco=%d, Speed=%.2f m/s",
                                current_segment_id_, marker_id, target_speed_);
                }
            }
        }
    }
    
    void update_segment_stats(int segment_id, bool success) {
        if (segment_id < 0 || segment_id >= static_cast<int>(segments_.size())) return;
        
        Segment& seg = segments_[segment_id];
        
        if (success) {
            seg.success_count++;
            seg.learned_speed = std::min(max_speed_, seg.learned_speed * speed_increase_rate_);
            
            if (segment_control_count_ > 10) { 
                double avg_derivative = segment_derivative_sum_ / segment_control_count_;
                
                if (avg_derivative < 0.005) {
                    seg.steering_multiplier *= 1.05;
                    seg.steering_multiplier = std::min(1.3, seg.steering_multiplier);
                    RCLCPP_INFO(get_logger(), "Seg %d: Very smooth (%.3f), steering↑ %.2f", 
                                segment_id, avg_derivative, seg.steering_multiplier);
                } else if (avg_derivative > 0.012) {
                    seg.steering_multiplier *= 0.95;
                    seg.steering_multiplier = std::max(0.7, seg.steering_multiplier);
                    RCLCPP_INFO(get_logger(), "Seg %d: Oscillating (%.3f), steering↓ %.2f", 
                                segment_id, avg_derivative, seg.steering_multiplier);
                } else {
                    RCLCPP_INFO(get_logger(), "Seg %d: Normal turn (%.3f), steering= %.2f", 
                                segment_id, avg_derivative, seg.steering_multiplier);
                }
            }
            
            RCLCPP_INFO(get_logger(), "Segment %d SUCCESS: speed=%.2f, steer=%.2f", 
                        segment_id, seg.learned_speed, seg.steering_multiplier);
        } else {
            seg.crash_count++;
            seg.learned_speed = std::max(min_speed_, seg.learned_speed * speed_decrease_rate_);
            seg.steering_multiplier *= 0.90;
            seg.steering_multiplier = std::max(0.7, seg.steering_multiplier);
            
            RCLCPP_WARN(get_logger(), "Segment %d CRASH: speed=%.2f, steer=%.2f", 
                        segment_id, seg.learned_speed, seg.steering_multiplier);
        }
    }
    
    bool get_robot_position() {
        try {
            auto tf = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
            current_x_ = tf.transform.translation.x;
            current_y_ = tf.transform.translation.y;
            return true;
        } catch (const tf2::TransformException&) {
            return false;
        }
    }
    
    double distance_from_start() {
        double dx = current_x_ - start_x_;
        double dy = current_y_ - start_y_;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    bool is_in_start_box() {
        return distance_from_start() < start_box_radius_;
    }
    
    bool has_left_start_area() {
        return distance_from_start() > min_leave_distance_;
    }
    
    void publish_state(const std::string& state) {
        auto msg = std_msgs::msg::String();
        msg.data = state;
        state_pub_->publish(msg);
    }
    
    void stop_vehicle() {
        geometry_msgs::msg::Twist cmd;
        cmd_vel_pub_->publish(cmd);
    }
    
    geometry_msgs::msg::Twist compute_cmd_vel() {
        geometry_msgs::msg::Twist cmd;
        
        double error = left_distance_ - right_distance_;
        
        double derivative = error - previous_error_;
        double base_steering = steering_kp_ * error + steering_kd_ * derivative;
        previous_error_ = error;
        
        double multiplier = 1.0;
        if (current_segment_id_ >= 0 && current_segment_id_ < static_cast<int>(segments_.size())) {
            multiplier = segments_[current_segment_id_].steering_multiplier;
        }
        double steering = base_steering * multiplier;
        
        segment_derivative_sum_ += std::abs(derivative);
        segment_control_count_++;
        
        steering = std::clamp(steering, -0.5, 0.5);
        
        double speed = target_speed_;
        if (front_distance_ < 1.0) {
            speed *= (front_distance_ / 1.0);
        }
        speed = std::max(min_speed_, speed);
        
        if (min_distance_ < crash_threshold_) {
            had_crash_this_segment_ = true;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                "Near crash! Min distance: %.2f", min_distance_);
        }
        
        cmd.linear.x = speed;
        cmd.angular.z = steering;
        
        return cmd;
    }
    
    void on_lap_complete() {
        double lap_time = (this->now() - lap_start_time_).seconds();
        lap_times_.push_back(lap_time);
        
        if (current_segment_id_ >= 0) {
            update_segment_stats(current_segment_id_, !had_crash_this_segment_);
        }
        
        RCLCPP_INFO(get_logger(), "===========================================");
        RCLCPP_INFO(get_logger(), "LAP %d COMPLETED! Time: %.2f s", current_lap_, lap_time);
        RCLCPP_INFO(get_logger(), "===========================================");
        
        save_segment_map();
        
        detected_markers_this_lap_.clear();
        current_segment_id_ = -1;
        had_crash_this_segment_ = false;
        has_left_start_ = false;
    }
    
    void print_results() {
        RCLCPP_INFO(get_logger(), "===========================================");
        RCLCPP_INFO(get_logger(), "   RACING COMPLETE - RESULTS");
        RCLCPP_INFO(get_logger(), "===========================================");
        
        for (size_t i = 0; i < lap_times_.size(); ++i) {
            RCLCPP_INFO(get_logger(), "Lap %zu: %.2f seconds", i + 1, lap_times_[i]);
        }
        
        if (lap_times_.size() >= 2) {
            double improvement = lap_times_[0] - lap_times_.back();
            double pct = (improvement / lap_times_[0]) * 100.0;
            RCLCPP_INFO(get_logger(), "-------------------------------------------");
            RCLCPP_INFO(get_logger(), "Improvement: %.2f seconds (%.1f%%)", improvement, pct);
        }
        
        RCLCPP_INFO(get_logger(), "-------------------------------------------");
        RCLCPP_INFO(get_logger(), "Learned segment speeds:");
        for (const auto& seg : segments_) {
            RCLCPP_INFO(get_logger(), "  Segment %d: %.2f m/s (success: %d, crash: %d)",
                        seg.id, seg.learned_speed, seg.success_count, seg.crash_count);
        }
        RCLCPP_INFO(get_logger(), "===========================================");
    }
    
    void control_loop() {
        if (!get_robot_position()) return;
        
        switch (current_state_) {
            case State::WAITING_START: {
                publish_state("WAITING_START");
                
                if (!start_recorded_) {
                    start_x_ = current_x_;
                    start_y_ = current_y_;
                    start_recorded_ = true;
                    current_lap_ = 1;
                    lap_start_time_ = this->now();
                    target_speed_ = default_speed_;
                    
                    RCLCPP_INFO(get_logger(), "RACING STARTED! Lap 1 of %d", total_laps_);
                    current_state_ = State::RACING;
                }
                break;
            }
            
            case State::RACING: {
                publish_state("RACING_LAP_" + std::to_string(current_lap_));
                
                auto cmd = compute_cmd_vel();
                cmd_vel_pub_->publish(cmd);
                
                if (!has_left_start_ && has_left_start_area()) {
                    has_left_start_ = true;
                    RCLCPP_INFO(get_logger(), "Left start area");
                }
                
                if (has_left_start_ && is_in_start_box()) {
                    on_lap_complete();
                    
                    if (current_lap_ >= total_laps_) {
                        current_state_ = State::FINISHED;
                    } else {
                        current_lap_++;
                        lap_start_time_ = this->now();
                        RCLCPP_INFO(get_logger(), "Starting Lap %d of %d", current_lap_, total_laps_);
                    }
                }
                break;
            }
            
            case State::LAP_COMPLETE: {
                break;
            }
            
            case State::FINISHED: {
                publish_state("FINISHED");
                stop_vehicle();
                print_results();
                
                RCLCPP_INFO(get_logger(), "Shutting down racing node...");
                rclcpp::shutdown();
                break;
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SegmentRacingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
