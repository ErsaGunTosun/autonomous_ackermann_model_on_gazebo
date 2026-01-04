#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include <cmath>
#include <cstdlib>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <vector>
#include <iomanip>
#include <set>
#include <algorithm>
#include <map>

enum class SegmentType {
    STRAIGHT,
    CURVE_LEFT,
    CURVE_RIGHT,
    SHARP_CURVE
};

struct Segment {
    int id;
    int start_aruco_id;
    int end_aruco_id;
    SegmentType type;
    double base_speed;
    double learned_speed;
    double x, y;
};

struct ActiveSegmentData {
    int start_marker_id;
    double start_heading;
    double start_x, start_y;
    std::vector<double> headings;
    bool is_active;
};

struct MarkerPassInfo {
    int marker_id;
    double heading_at_pass;
    double cumulative_heading_change;
    double distance;
    double x, y;
};

class FormationLapNode : public rclcpp::Node 
{
public:
    enum class State {
        IDLE,
        FORMATION_LAP,
        WAITING_LOOP_CLOSURE,
        SAVING_MAP,
        COMPLETE
    };

    FormationLapNode() : Node("formation_lap_node") {
        current_state_ = State::IDLE;
        loop_closure_wait_time_ = 10.0;
    
        start_recorded_ = false;
        has_left_start_ = false;
        map_saved_ = false;
        segments_saved_ = false;
        complete_time_set_ = false;
        
        active_segment_.is_active = false;
        
        first_marker_id_ = -1;
        marker_0_seen_count_ = 0;
        lap_completed_ = false;
        
        current_marker_id_ = -1;
        current_marker_min_distance_ = 999.0;
        distance_increasing_count_ = 0;
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        lane_follow_pub_ = create_publisher<std_msgs::msg::Bool>("/control/lane_follow", 10);
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        state_pub_ = create_publisher<std_msgs::msg::String>("/formation_lap/state", 10);
        
        aruco_ids_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "/aruco/detected_ids",
            10,
            std::bind(&FormationLapNode::aruco_ids_callback, this, std::placeholders::_1));
        
        aruco_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco/pose",
            10,
            std::bind(&FormationLapNode::aruco_pose_callback, this, std::placeholders::_1));
        
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&FormationLapNode::formation_control, this));
        
        RCLCPP_INFO(get_logger(), "===========================================");
        RCLCPP_INFO(get_logger(), "   FORMATION LAP NODE (Marker-based)");
        RCLCPP_INFO(get_logger(), "===========================================");
    }

private:
    State current_state_;
    double loop_closure_wait_time_;
    
    double start_x_, start_y_;
    bool start_recorded_;
    bool has_left_start_;
    bool map_saved_;
    bool segments_saved_;
    bool complete_time_set_;
    
    std::vector<Segment> segments_;
    std::vector<MarkerPassInfo> passed_markers_;
    ActiveSegmentData active_segment_;
    
    int first_marker_id_;
    int marker_0_seen_count_;
    bool lap_completed_;
    
    int current_marker_id_;
    double current_marker_distance_;
    double current_marker_min_distance_;
    double current_marker_heading_at_min_;
    double current_marker_x_at_min_;
    double current_marker_y_at_min_;
    int distance_increasing_count_;
    
    double odom_x_, odom_y_, odom_yaw_;
    double prev_odom_yaw_;
    bool prev_yaw_valid_ = false;
    
    rclcpp::Time loop_closure_start_time_;
    rclcpp::Time complete_start_time_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lane_follow_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr aruco_ids_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle <= -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void aruco_ids_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (current_state_ != State::FORMATION_LAP) return;
        
        if (msg->data.empty()) {
            if (current_marker_id_ >= 0 && current_marker_min_distance_ < 1.5) {
                RCLCPP_INFO(get_logger(), "Marker %d lost at close range (%.2fm), finalizing", 
                           current_marker_id_, current_marker_min_distance_);
                finalize_marker_pass();
                current_marker_id_ = -1;
            }
            return;
        }
        
        int closest_marker_id = msg->data[0];
        update_current_marker(closest_marker_id);
    }
    
    void aruco_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (current_state_ != State::FORMATION_LAP) return;
        
        double distance = std::sqrt(
            msg->pose.position.x * msg->pose.position.x +
            msg->pose.position.y * msg->pose.position.y +
            msg->pose.position.z * msg->pose.position.z);
        
        current_marker_distance_ = distance;
        
        if (distance < current_marker_min_distance_) {
            current_marker_min_distance_ = distance;
            current_marker_heading_at_min_ = odom_yaw_;
            current_marker_x_at_min_ = odom_x_;
            current_marker_y_at_min_ = odom_y_;
            distance_increasing_count_ = 0;
        } else {
            distance_increasing_count_++;
            if (distance_increasing_count_ > 5 && current_marker_id_ >= 0) {
                finalize_marker_pass();
            }
        }
    }
    
    void update_current_marker(int marker_id) {
        if (marker_id != current_marker_id_) {
            if (current_marker_id_ >= 0) {
                finalize_marker_pass();
            }
            
            current_marker_id_ = marker_id;
            current_marker_min_distance_ = 999.0;
            distance_increasing_count_ = 0;
            
            RCLCPP_INFO(get_logger(), "Tracking marker %d", marker_id);
            
            if (first_marker_id_ >= 0 && marker_id == first_marker_id_ && marker_0_seen_count_ >= 1) {
                RCLCPP_INFO(get_logger(), "============================================");
                RCLCPP_INFO(get_logger(), "MARKER %d SEEN AGAIN - LAP COMPLETE!", first_marker_id_);
                
                if (active_segment_.is_active) {
                    complete_segment();
                }
                
                lap_completed_ = true;
                set_lane_follow(false);
                stop_vehicle();
                current_state_ = State::WAITING_LOOP_CLOSURE;
                loop_closure_start_time_ = this->now();
            }
        }
    }
    
    void finalize_marker_pass() {
        if (current_marker_id_ < 0) return;
        if (current_marker_min_distance_ > 2.0) return;
        
        if (first_marker_id_ < 0) {
            first_marker_id_ = current_marker_id_;
            RCLCPP_INFO(get_logger(), "First marker: %d", first_marker_id_);
        }
        if (current_marker_id_ == first_marker_id_) {
            marker_0_seen_count_++;
            RCLCPP_INFO(get_logger(), "First marker seen %d times", marker_0_seen_count_);
            
            if (marker_0_seen_count_ > 1 && !lap_completed_) {
                RCLCPP_INFO(get_logger(), "============================================");
                RCLCPP_INFO(get_logger(), "MARKER %d SEEN AGAIN - LAP COMPLETE!", first_marker_id_);
                if (active_segment_.is_active) {
                    complete_segment();
                }
                lap_completed_ = true;
                set_lane_follow(false);
                stop_vehicle();
                current_state_ = State::WAITING_LOOP_CLOSURE;
                loop_closure_start_time_ = this->now();
                return;
            }
        }
        
        for (const auto& mp : passed_markers_) {
            if (mp.marker_id == current_marker_id_) {
                RCLCPP_DEBUG(get_logger(), "Marker %d already recorded", current_marker_id_);
                return;
            }
        }
        
        if (active_segment_.is_active) {
            complete_segment();
        }
        
        start_new_segment();
        
        current_marker_min_distance_ = 999.0;
        distance_increasing_count_ = 0;
    }
    
    void start_new_segment() {
        active_segment_.start_marker_id = current_marker_id_;
        active_segment_.start_heading = current_marker_heading_at_min_;
        active_segment_.start_x = current_marker_x_at_min_;
        active_segment_.start_y = current_marker_y_at_min_;
        active_segment_.headings.clear();
        active_segment_.headings.push_back(current_marker_heading_at_min_);
        active_segment_.is_active = true;
        
        RCLCPP_INFO(get_logger(), 
            "MARKER %d PASSED: heading=%.1f°, Segment STARTED",
            current_marker_id_, current_marker_heading_at_min_ * 180.0 / M_PI);
        
        MarkerPassInfo info;
        info.marker_id = current_marker_id_;
        info.heading_at_pass = current_marker_heading_at_min_;
        info.x = current_marker_x_at_min_;
        info.y = current_marker_y_at_min_;
        passed_markers_.push_back(info);
    }
    
    void complete_segment() {
        if (!active_segment_.is_active) return;
        if (active_segment_.headings.size() < 5) return;
        
        double start_heading = active_segment_.start_heading;
        double end_heading = current_marker_heading_at_min_;
        double end_x = current_marker_x_at_min_;
        double end_y = current_marker_y_at_min_;
        
        double net_change = normalize_angle(end_heading - start_heading);
        double net_change_deg = net_change * 180.0 / M_PI;
        
        double max_dev = 0.0;
        double min_dev = 0.0;
        for (const auto& h : active_segment_.headings) {
            double dev = normalize_angle(h - start_heading) * 180.0 / M_PI;
            max_dev = std::max(max_dev, dev);
            min_dev = std::min(min_dev, dev);
        }
        double range_deg = max_dev - min_dev;
        
        double abs_net = std::abs(net_change_deg);
        bool high_oscillation = (abs_net > 5.0) && (range_deg > 2.5 * abs_net);
        bool both_directions = (max_dev > 50.0) && (min_dev < -50.0);
        bool is_s_curve = high_oscillation && both_directions;
        
        double dx = end_x - active_segment_.start_x;
        double dy = end_y - active_segment_.start_y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        Segment seg;
        seg.id = segments_.size();
        seg.start_aruco_id = active_segment_.start_marker_id;
        seg.end_aruco_id = current_marker_id_;
        seg.type = determine_segment_type(net_change_deg, is_s_curve);
        seg.base_speed = get_base_speed(seg.type);
        seg.learned_speed = seg.base_speed;
        seg.x = active_segment_.start_x;
        seg.y = active_segment_.start_y;
        
        segments_.push_back(seg);
        
        RCLCPP_INFO(get_logger(), 
            "SEGMENT %d: ArUco %d→%d | net=%.1f° range=%.1f° S=%d | Type=%s",
            seg.id, seg.start_aruco_id, seg.end_aruco_id,
            net_change_deg, range_deg, is_s_curve,
            segment_type_to_string(seg.type).c_str());
    }
    
    SegmentType determine_segment_type(double net_deg, bool is_s_curve) {
        double abs_net = std::abs(net_deg);
        
        if (is_s_curve) {
            return SegmentType::SHARP_CURVE;
        }
        
        if (abs_net > 75.0) {
            return SegmentType::SHARP_CURVE;
        }
        
        if (abs_net < 50.0) {
            return SegmentType::STRAIGHT;
        }
        
        if (net_deg > 0) {
            return SegmentType::CURVE_LEFT;
        } else {
            return SegmentType::CURVE_RIGHT;
        }
    }
    
    double get_base_speed(SegmentType type) {
        switch (type) {
            case SegmentType::STRAIGHT:    return 1.2;  // 0.5 → 1.2
            case SegmentType::CURVE_LEFT:  return 0.8;  // 0.35 → 0.8
            case SegmentType::CURVE_RIGHT: return 0.8;  // 0.35 → 0.8
            case SegmentType::SHARP_CURVE: return 0.5;  // 0.25 → 0.5
            default: return 0.6;
        }
    }
    
    std::string segment_type_to_string(SegmentType type) {
        switch (type) {
            case SegmentType::STRAIGHT:    return "STRAIGHT";
            case SegmentType::CURVE_LEFT:  return "CURVE_LEFT";
            case SegmentType::CURVE_RIGHT: return "CURVE_RIGHT";
            case SegmentType::SHARP_CURVE: return "SHARP_CURVE";
            default: return "UNKNOWN";
        }
    }

    bool get_robot_position() {
        try {
            auto odom_tf = tf_buffer_->lookupTransform(
                "odom", "base_footprint", tf2::TimePointZero);
            odom_x_ = odom_tf.transform.translation.x;
            odom_y_ = odom_tf.transform.translation.y;
            
            tf2::Quaternion q_odom(
                odom_tf.transform.rotation.x,
                odom_tf.transform.rotation.y,
                odom_tf.transform.rotation.z,
                odom_tf.transform.rotation.w);
            odom_yaw_ = tf2::getYaw(q_odom);
            
            return true;
        } catch (const tf2::TransformException &ex) {
            return false;
        }
    }
    
    void collect_heading() {
        if (current_state_ == State::FORMATION_LAP && active_segment_.is_active) {
            active_segment_.headings.push_back(odom_yaw_);
        }
    }
    
    void save_segments() {
        
        if (segments_.empty()) {
            RCLCPP_WARN(get_logger(), "No segments to save!");
            segments_saved_ = true;
            return;
        }
        
        std::string nav_pkg_path = ament_index_cpp::get_package_share_directory("vehicle_navigation");
        std::string maps_dir = nav_pkg_path + "/maps";
        
        if (!std::filesystem::exists(maps_dir)) {
            std::filesystem::create_directories(maps_dir);
        }
        
        std::string segments_path = maps_dir + "/segment_map.yaml";
        std::ofstream file(segments_path);
        
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open: %s", segments_path.c_str());
            return;
        }
        
        file << "# Segment map - FUSION detection\n";
        file << "# Total segments: " << segments_.size() << "\n\n";
        file << "segments:\n";
        
        for (const auto& seg : segments_) {
            file << "  - id: " << seg.id << "\n";
            file << "    start_aruco_id: " << seg.start_aruco_id << "\n";
            file << "    end_aruco_id: " << seg.end_aruco_id << "\n";
            file << "    type: " << segment_type_to_string(seg.type) << "\n";
            file << "    base_speed: " << std::fixed << std::setprecision(2) << seg.base_speed << "\n";
            file << "    learned_speed: " << std::fixed << std::setprecision(2) << seg.learned_speed << "\n";
            file << "    success_count: 0\n";
            file << "    crash_count: 0\n";
        }
        
        file.close();
        segments_saved_ = true;
        RCLCPP_INFO(get_logger(), "Saved %zu segments to: %s", segments_.size(), segments_path.c_str());
    }

    void publish_state(const std::string& state_name) {
        auto msg = std_msgs::msg::String();
        msg.data = state_name;
        state_pub_->publish(msg);
    }

    void set_lane_follow(bool enable) {
        auto msg = std_msgs::msg::Bool();
        msg.data = enable;
        lane_follow_pub_->publish(msg);
    }

    void stop_vehicle() {
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
    }

    void save_map_to_file() {
        RCLCPP_INFO(get_logger(), "Saving map...");
        std::string nav_pkg_path = ament_index_cpp::get_package_share_directory("vehicle_navigation");
        std::string map_path = nav_pkg_path + "/maps/formation_map";
        std::string cmd = "ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "
                         "\"{name: {data: '" + map_path + "'}}\"";
        int result = std::system(cmd.c_str());
        if (result == 0) {
            RCLCPP_INFO(get_logger(), "Map saved");
            map_saved_ = true;
        } else {
            RCLCPP_ERROR(get_logger(), "Map save failed!");
        }
    }

    void formation_control() {
        if (!get_robot_position()) return;
        
        collect_heading();
        
        switch (current_state_) {
            case State::IDLE:
                publish_state("IDLE");
                if (!start_recorded_) {
                    start_x_ = odom_x_;
                    start_y_ = odom_y_;
                    start_recorded_ = true;
                    RCLCPP_INFO(get_logger(), "Formation lap starting at (%.2f, %.2f)", 
                               start_x_, start_y_);
                    current_state_ = State::FORMATION_LAP;
                }
                break;

            case State::FORMATION_LAP:
                publish_state("FORMATION_LAP");
                set_lane_follow(true);
                
                if (!has_left_start_ && first_marker_id_ >= 0) {
                    has_left_start_ = true;
                    RCLCPP_INFO(get_logger(), "First marker detected, lap recording started");
                }
                break;

            case State::WAITING_LOOP_CLOSURE:
                publish_state("WAITING_LOOP_CLOSURE");
                stop_vehicle();
                if ((this->now() - loop_closure_start_time_).seconds() >= loop_closure_wait_time_) {
                    current_state_ = State::SAVING_MAP;
                }
                break;

            case State::SAVING_MAP:
                publish_state("SAVING_MAP");
                stop_vehicle();
                if (!map_saved_) save_map_to_file();
                if (map_saved_ && !segments_saved_) save_segments();
                if (map_saved_ && segments_saved_) {
                    RCLCPP_INFO(get_logger(), "Complete! %zu segments saved.", segments_.size());
                    current_state_ = State::COMPLETE;
                }
                break;

            case State::COMPLETE:
                publish_state("COMPLETE");
                stop_vehicle();
                if (!complete_time_set_) {
                    complete_start_time_ = this->now();
                    complete_time_set_ = true;
                }
                if ((this->now() - complete_start_time_).seconds() >= 3.0) {
                    rclcpp::shutdown();
                }
                break;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FormationLapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
