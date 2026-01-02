#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
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

// Waypoint yapısı
struct Waypoint {
    double x;
    double y;
    double theta;
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
        
        start_box_radius_ = 1.5;
        min_leave_distance_ = 3.0;
        loop_closure_wait_time_ = 10.0;
    
        start_recorded_ = false;
        has_left_start_ = false;
        map_saved_ = false;
        waypoints_saved_ = false;
        complete_time_set_ = false;
        first_waypoint_recorded_ = false;
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        lane_follow_pub_ = create_publisher<std_msgs::msg::Bool>("/control/lane_follow", 10);
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        state_pub_ = create_publisher<std_msgs::msg::String>("/formation_lap/state", 10);
        
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&FormationLapNode::formation_control, this));
        
        RCLCPP_INFO(get_logger(), "===========================================");
        RCLCPP_INFO(get_logger(), "   FORMATION LAP NODE STARTED");
        RCLCPP_INFO(get_logger(), "===========================================");
    }

private:
    State current_state_;
    
    double start_box_radius_;
    double min_leave_distance_;
    double loop_closure_wait_time_;
    
    double start_x_, start_y_;
    double current_x_, current_y_, current_yaw_;
    bool start_recorded_;
    bool has_left_start_;
    bool map_saved_;
    bool waypoints_saved_;
    bool complete_time_set_;
    
    // Waypoint kaydetme için değişkenler
    std::vector<Waypoint> waypoints_;
    double waypoint_interval_ = 0.5;  // metre
    double last_waypoint_x_, last_waypoint_y_;
    bool first_waypoint_recorded_;
    
    rclcpp::Time loop_closure_start_time_;
    rclcpp::Time complete_start_time_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lane_follow_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool get_robot_position() {
        try {
            auto transform = tf_buffer_->lookupTransform(
                "map", "base_footprint", tf2::TimePointZero);
            current_x_ = transform.transform.translation.x;
            current_y_ = transform.transform.translation.y;
            
            // Quaternion'dan yaw açısını çıkar
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
            current_yaw_ = tf2::getYaw(q);
            
            return true;
        } catch (const tf2::TransformException &ex) {
            return false;
        }
    }

    // Son waypoint'ten uzaklığı hesapla
    double distance_from_last_waypoint() {
        double dx = current_x_ - last_waypoint_x_;
        double dy = current_y_ - last_waypoint_y_;
        return std::sqrt(dx * dx + dy * dy);
    }

    // Waypoint kaydet
    void record_waypoint() {
        Waypoint wp;
        wp.x = current_x_;
        wp.y = current_y_;
        wp.theta = current_yaw_;
        waypoints_.push_back(wp);
        
        last_waypoint_x_ = current_x_;
        last_waypoint_y_ = current_y_;
        
        RCLCPP_DEBUG(get_logger(), "Waypoint %zu recorded: (%.2f, %.2f, %.2f)", 
                     waypoints_.size(), wp.x, wp.y, wp.theta);
    }

    // Waypoint'leri YAML dosyasına kaydet
    void save_waypoints_to_file() {
        std::string nav_pkg_path = ament_index_cpp::get_package_share_directory("vehicle_navigation");
        std::string maps_dir = nav_pkg_path + "/maps";
        
        if (!std::filesystem::exists(maps_dir)) {
            std::filesystem::create_directories(maps_dir);
        }
        
        std::string waypoints_path = maps_dir + "/formation_waypoints.yaml";
        std::ofstream file(waypoints_path);
        
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open waypoints file: %s", waypoints_path.c_str());
            return;
        }
        
        file << "# Waypoints generated by formation_lap_node\n";
        file << "# Total waypoints: " << waypoints_.size() << "\n";
        file << "# Interval: " << waypoint_interval_ << " meters\n\n";
        file << "waypoints:\n";
        
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            file << "  - id: " << i << "\n";
            file << "    x: " << std::fixed << std::setprecision(4) << waypoints_[i].x << "\n";
            file << "    y: " << std::fixed << std::setprecision(4) << waypoints_[i].y << "\n";
            file << "    theta: " << std::fixed << std::setprecision(4) << waypoints_[i].theta << "\n";
        }
        
        file.close();
        waypoints_saved_ = true;
        RCLCPP_INFO(get_logger(), "Saved %zu waypoints to: %s", waypoints_.size(), waypoints_path.c_str());
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
        
        // Map dosyasını vehicle_navigation/maps/ klasörüne kaydet
        std::string nav_pkg_path = ament_index_cpp::get_package_share_directory("vehicle_navigation");
        std::string maps_dir = nav_pkg_path + "/maps";
        
        // Klasör yoksa oluştur
        if (!std::filesystem::exists(maps_dir)) {
            std::filesystem::create_directories(maps_dir);
        }
        
        std::string map_path = maps_dir + "/formation_map";
        
        std::string cmd = "ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "
                         "\"{name: {data: '" + map_path + "'}}\"";
        
        int result = std::system(cmd.c_str());
        
        if (result == 0) {
            RCLCPP_INFO(get_logger(), "Map saved to: %s", map_path.c_str());
            map_saved_ = true;
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to save map!");
        }
    }

    void formation_control() {
        if (!get_robot_position()) return;
        
        switch (current_state_) {
            case State::IDLE: {
                publish_state("IDLE");
                if (!start_recorded_) {
                    start_x_ = current_x_;
                    start_y_ = current_y_;
                    start_recorded_ = true;
                    has_left_start_ = false;
                    
                    // İlk waypoint'i kaydet
                    record_waypoint();
                    first_waypoint_recorded_ = true;
                    
                    RCLCPP_INFO(get_logger(), "FORMATION LAP STARTING - Start: (%.2f, %.2f)", start_x_, start_y_);
                    current_state_ = State::FORMATION_LAP;
                }
                break;
            }

            case State::FORMATION_LAP: {
                publish_state("FORMATION_LAP");
                set_lane_follow(true);
                
                // Waypoint kaydet (0.5m aralıklarla)
                if (distance_from_last_waypoint() >= waypoint_interval_) {
                    record_waypoint();
                }
                
                if (!has_left_start_ && has_left_start_area()) {
                    has_left_start_ = true;
                    RCLCPP_INFO(get_logger(), "Left start area! Distance: %.2f, Waypoints: %zu", 
                                distance_from_start(), waypoints_.size());
                }
                
                if (has_left_start_ && is_in_start_box()) {
                    // Son waypoint'i başlangıç noktasına yakın kaydet
                    record_waypoint();
                    
                    RCLCPP_INFO(get_logger(), "===========================================");
                    RCLCPP_INFO(get_logger(), "FORMATION LAP COMPLETED! Total waypoints: %zu", waypoints_.size());
                    RCLCPP_INFO(get_logger(), "===========================================");
                    set_lane_follow(false);
                    stop_vehicle();
                    current_state_ = State::WAITING_LOOP_CLOSURE;
                    loop_closure_start_time_ = this->now();
                }
                break;
            }

            case State::WAITING_LOOP_CLOSURE: {
                publish_state("WAITING_LOOP_CLOSURE");
                stop_vehicle();
                
                double elapsed = (this->now() - loop_closure_start_time_).seconds();
                if (elapsed >= loop_closure_wait_time_) {
                    current_state_ = State::SAVING_MAP;
                }
                break;
            }

            case State::SAVING_MAP: {
                publish_state("SAVING_MAP");
                stop_vehicle();
                
                if (!map_saved_) {
                    save_map_to_file();
                }
                
                // Waypoint'leri de kaydet
                if (map_saved_ && !waypoints_saved_) {
                    save_waypoints_to_file();
                }
                
                if (map_saved_ && waypoints_saved_) {
                    RCLCPP_INFO(get_logger(), "===========================================");
                    RCLCPP_INFO(get_logger(), "FORMATION LAP COMPLETE!");
                    RCLCPP_INFO(get_logger(), "Map and %zu waypoints saved.", waypoints_.size());
                    RCLCPP_INFO(get_logger(), "You can now start racing launch.");
                    RCLCPP_INFO(get_logger(), "===========================================");
                    current_state_ = State::COMPLETE;
                }
                break;
            }

            case State::COMPLETE: {
                publish_state("COMPLETE");
                stop_vehicle();
                
                // İlk kez COMPLETE state'ine girdiyse zamanı kaydet
                if (!complete_time_set_) {
                    complete_start_time_ = this->now();
                    complete_time_set_ = true;
                }
                
                // 3 saniye bekleyip node'u kapat
                double elapsed = (this->now() - complete_start_time_).seconds();
                if (elapsed >= 3.0) {
                    RCLCPP_INFO(get_logger(), "Shutting down formation lap node...");
                    rclcpp::shutdown();
                }
                break;
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FormationLapNode>());
    rclcpp::shutdown();
    return 0;
}

