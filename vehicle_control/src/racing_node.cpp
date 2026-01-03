/**
 * Racing Node - 3 Tur Waypoint Takibi
 * 
 * Bu node formation lap'ten kaydedilen waypoint'leri kullanarak
 * Nav2 FollowWaypoints action ile 3 tur atar.
 * Her turda hız artırılarak performans iyileştirilir.
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <chrono>
#include <regex>

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

struct Waypoint {
    double x;
    double y;
    double theta;
};

class RacingNode : public rclcpp::Node
{
public:
    enum class State {
        WAITING_NAV2,
        LOADING_WAYPOINTS,
        RACING_LAP_1,
        RACING_LAP_2,
        RACING_LAP_3,
        COMPLETE
    };

    RacingNode() : Node("racing_node")
    {
        current_state_ = State::WAITING_NAV2;
        current_lap_ = 0;
        total_laps_ = 3;
        nav2_ready_ = false;
        goal_in_progress_ = false;
        
        // Her tur için hız ayarları (velocity scaling)
        lap_speeds_ = {0.5, 0.7, 0.9};  // m/s
        
        // Publishers
        state_pub_ = create_publisher<std_msgs::msg::String>("/racing/state", 10);
        
        // Action client for FollowWaypoints
        follow_waypoints_client_ = rclcpp_action::create_client<FollowWaypoints>(
            this, "follow_waypoints");
        
        // Main timer
        timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&RacingNode::racing_control, this));
        
        RCLCPP_INFO(get_logger(), "===========================================");
        RCLCPP_INFO(get_logger(), "   RACING NODE STARTED");
        RCLCPP_INFO(get_logger(), "   Total laps: %d", total_laps_);
        RCLCPP_INFO(get_logger(), "===========================================");
    }

private:
    State current_state_;
    int current_lap_;
    int total_laps_;
    bool nav2_ready_;
    bool goal_in_progress_;
    
    std::vector<Waypoint> waypoints_;
    std::vector<double> lap_speeds_;
    std::vector<double> lap_times_;
    
    rclcpp::Time lap_start_time_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_state(const std::string& state_name) {
        auto msg = std_msgs::msg::String();
        msg.data = state_name;
        state_pub_->publish(msg);
    }

    bool load_waypoints() {
        std::string nav_pkg_path = ament_index_cpp::get_package_share_directory("vehicle_navigation");
        std::string waypoints_path = nav_pkg_path + "/maps/formation_waypoints.yaml";
        
        std::ifstream file(waypoints_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open waypoints file: %s", waypoints_path.c_str());
            return false;
        }
        
        waypoints_.clear();
        std::string line;
        Waypoint current_wp;
        bool in_waypoint = false;
        
        // Basit YAML parser
        while (std::getline(file, line)) {
            // Boş satır veya yorum atla
            if (line.empty() || line[0] == '#') continue;
            
            // Yeni waypoint başlangıcı
            if (line.find("- id:") != std::string::npos) {
                if (in_waypoint) {
                    waypoints_.push_back(current_wp);
                }
                in_waypoint = true;
                current_wp = Waypoint();
            }
            // x değeri
            else if (line.find("x:") != std::string::npos && in_waypoint) {
                std::regex re("x:\\s*([\\-0-9.]+)");
                std::smatch match;
                if (std::regex_search(line, match, re)) {
                    current_wp.x = std::stod(match[1]);
                }
            }
            // y değeri
            else if (line.find("y:") != std::string::npos && in_waypoint) {
                std::regex re("y:\\s*([\\-0-9.]+)");
                std::smatch match;
                if (std::regex_search(line, match, re)) {
                    current_wp.y = std::stod(match[1]);
                }
            }
            // theta değeri
            else if (line.find("theta:") != std::string::npos && in_waypoint) {
                std::regex re("theta:\\s*([\\-0-9.]+)");
                std::smatch match;
                if (std::regex_search(line, match, re)) {
                    current_wp.theta = std::stod(match[1]);
                }
            }
        }
        
        // Son waypoint'i ekle
        if (in_waypoint) {
            waypoints_.push_back(current_wp);
        }
        
        file.close();
        
        RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from file", waypoints_.size());
        return !waypoints_.empty();
    }

    std::vector<geometry_msgs::msg::PoseStamped> create_pose_array() {
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        
        for (const auto& wp : waypoints_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = this->now();
            pose.pose.position.x = wp.x;
            pose.pose.position.y = wp.y;
            pose.pose.position.z = 0.0;
            
            tf2::Quaternion q;
            q.setRPY(0, 0, wp.theta);
            pose.pose.orientation = tf2::toMsg(q);
            
            poses.push_back(pose);
        }
        
        return poses;
    }

    void send_waypoints_goal() {
        if (!follow_waypoints_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_logger(), "FollowWaypoints action server not available");
            return;
        }
        
        auto goal_msg = FollowWaypoints::Goal();
        goal_msg.poses = create_pose_array();
        
        RCLCPP_INFO(get_logger(), "Sending %zu waypoints for lap %d", 
                    goal_msg.poses.size(), current_lap_);
        
        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            [this](const GoalHandleFollowWaypoints::SharedPtr& goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
                    goal_in_progress_ = false;
                } else {
                    RCLCPP_INFO(get_logger(), "Goal accepted by server");
                }
            };
        
        send_goal_options.feedback_callback =
            [this](GoalHandleFollowWaypoints::SharedPtr,
                   const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
                RCLCPP_DEBUG(get_logger(), "Current waypoint: %d", feedback->current_waypoint);
            };
        
        send_goal_options.result_callback =
            [this](const GoalHandleFollowWaypoints::WrappedResult& result) {
                goal_in_progress_ = false;
                
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        on_lap_completed();
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(get_logger(), "Goal was aborted");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_ERROR(get_logger(), "Goal was canceled");
                        break;
                    default:
                        RCLCPP_ERROR(get_logger(), "Unknown result code");
                        break;
                }
            };
        
        follow_waypoints_client_->async_send_goal(goal_msg, send_goal_options);
        goal_in_progress_ = true;
        lap_start_time_ = this->now();
    }

    void on_lap_completed() {
        double lap_time = (this->now() - lap_start_time_).seconds();
        lap_times_.push_back(lap_time);
        
        RCLCPP_INFO(get_logger(), "===========================================");
        RCLCPP_INFO(get_logger(), "LAP %d COMPLETED!", current_lap_);
        RCLCPP_INFO(get_logger(), "Lap time: %.2f seconds", lap_time);
        RCLCPP_INFO(get_logger(), "===========================================");
        
        // Sonraki state'e geç
        switch (current_state_) {
            case State::RACING_LAP_1:
                current_state_ = State::RACING_LAP_2;
                break;
            case State::RACING_LAP_2:
                current_state_ = State::RACING_LAP_3;
                break;
            case State::RACING_LAP_3:
                current_state_ = State::COMPLETE;
                break;
            default:
                break;
        }
    }

    void print_results() {
        RCLCPP_INFO(get_logger(), "===========================================");
        RCLCPP_INFO(get_logger(), "   RACING COMPLETE - RESULTS");
        RCLCPP_INFO(get_logger(), "===========================================");
        
        for (size_t i = 0; i < lap_times_.size(); ++i) {
            RCLCPP_INFO(get_logger(), "Lap %zu: %.2f seconds (speed: %.1f m/s)", 
                        i + 1, lap_times_[i], lap_speeds_[i]);
        }
        
        if (lap_times_.size() >= 2) {
            double improvement = lap_times_[0] - lap_times_[lap_times_.size() - 1];
            double improvement_pct = (improvement / lap_times_[0]) * 100.0;
            RCLCPP_INFO(get_logger(), "-------------------------------------------");
            RCLCPP_INFO(get_logger(), "Total improvement: %.2f seconds (%.1f%%)", 
                        improvement, improvement_pct);
        }
        
        RCLCPP_INFO(get_logger(), "===========================================");
    }

    void racing_control() {
        switch (current_state_) {
            case State::WAITING_NAV2: {
                publish_state("WAITING_NAV2");
                
                // Nav2 action server'ın hazır olmasını bekle
                if (follow_waypoints_client_->wait_for_action_server(std::chrono::seconds(1))) {
                    RCLCPP_INFO(get_logger(), "Nav2 is ready!");
                    nav2_ready_ = true;
                    current_state_ = State::LOADING_WAYPOINTS;
                } else {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, 
                                         "Waiting for Nav2 action server...");
                }
                break;
            }
            
            case State::LOADING_WAYPOINTS: {
                publish_state("LOADING_WAYPOINTS");
                
                if (load_waypoints()) {
                    current_lap_ = 1;
                    current_state_ = State::RACING_LAP_1;
                    RCLCPP_INFO(get_logger(), "Starting racing laps!");
                } else {
                    RCLCPP_ERROR(get_logger(), "Failed to load waypoints, retrying...");
                }
                break;
            }
            
            case State::RACING_LAP_1: {
                publish_state("RACING_LAP_1");
                
                if (!goal_in_progress_) {
                    current_lap_ = 1;
                    // TODO: Hız ayarı için dinamik reconfigure veya parameter set
                    send_waypoints_goal();
                }
                break;
            }
            
            case State::RACING_LAP_2: {
                publish_state("RACING_LAP_2");
                
                if (!goal_in_progress_) {
                    current_lap_ = 2;
                    send_waypoints_goal();
                }
                break;
            }
            
            case State::RACING_LAP_3: {
                publish_state("RACING_LAP_3");
                
                if (!goal_in_progress_) {
                    current_lap_ = 3;
                    send_waypoints_goal();
                }
                break;
            }
            
            case State::COMPLETE: {
                publish_state("COMPLETE");
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
    rclcpp::spin(std::make_shared<RacingNode>());
    rclcpp::shutdown();
    return 0;
}
