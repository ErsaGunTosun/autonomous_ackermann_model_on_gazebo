#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include <cmath>

class MissionManager : public rclcpp::Node 
{
public:
    enum class State {
        IDLE,
        FORMATION_LAP,
        WAITING_AT_START,
        RACING,
        COMPLETED
    };

    MissionManager() : Node("mission_manager") {
        current_state_ = State::IDLE;
        lap_count_ = 0;
        
        start_box_radius_ = 1.0;
        min_leave_distance_ = 2.0;   
        wait_duration_ = 5.0;        
    
        start_recorded_ = false;
        has_left_start_ = false;
        start_x_ = 0.0;
        start_y_ = 0.0;
        current_x_ = 0.0;
        current_y_ = 0.0;
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        lane_follow_pub_ = create_publisher<std_msgs::msg::Bool>("/control/lane_follow", 10);
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&MissionManager::mission_control, this));
        
        RCLCPP_INFO(get_logger(), "Mission Manager Started!");
        RCLCPP_INFO(get_logger(), "Start box radius: %.1f m, Min leave distance: %.1f m", 
            start_box_radius_, min_leave_distance_);
    }

private:
    State current_state_;
    int lap_count_;
    
    double start_box_radius_;
    double min_leave_distance_;
    double wait_duration_;
    
    double start_x_, start_y_;
    double current_x_, current_y_;
    bool start_recorded_;
    bool has_left_start_;
    
    rclcpp::Time wait_start_time_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lane_follow_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool get_robot_position() {
        try {
            auto transform = tf_buffer_->lookupTransform(
                "map", "base_footprint", tf2::TimePointZero);
            
            current_x_ = transform.transform.translation.x;
            current_y_ = transform.transform.translation.y;
            return true;
        } catch (const tf2::TransformException &ex) {
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

    void mission_control() {
        if (!get_robot_position())
        return;  

        
        switch (current_state_) {
            case State::IDLE: {
                if (!start_recorded_) {
                    start_x_ = current_x_;
                    start_y_ = current_y_;
                    start_recorded_ = true;
                    current_state_ = State::FORMATION_LAP;
                    RCLCPP_INFO(get_logger(), "FORMATION LAP STARTED");
                }
                break;
            }

            case State::FORMATION_LAP: {
                std_msgs::msg::Bool msg;
                msg.data = true;
                lane_follow_pub_->publish(msg);
                
                double dist = distance_from_start();
                
                if (!has_left_start_ && has_left_start_area()) {
                    has_left_start_ = true;
                    RCLCPP_INFO(get_logger(), "Left start area! Distance: %.2f m", dist);
                }
                
                if (has_left_start_ && is_in_start_box()) {
                    lap_count_++;
                    RCLCPP_INFO(get_logger(), "=== LAP %d COMPLETED! Distance: %.2f m ===", lap_count_, dist);
                    
                    current_state_ = State::WAITING_AT_START;
                    wait_start_time_ = this->now();
                    
                    std_msgs::msg::Bool stop_msg;
                    stop_msg.data = false;
                    lane_follow_pub_->publish(stop_msg);
                    
                    RCLCPP_INFO(get_logger(), "RACİNG AT START");
                }
                break;
            }

            case State::WAITING_AT_START: {
                geometry_msgs::msg::Twist stop_cmd;
                stop_cmd.linear.x = 0.0;
                stop_cmd.angular.z = 0.0;
                cmd_vel_pub_->publish(stop_cmd);
                
                double elapsed = (this->now() - wait_start_time_).seconds();
                
                if (elapsed >= wait_duration_) {
                    current_state_ = State::RACING;
                    has_left_start_ = false;  
                    RCLCPP_INFO(get_logger(), "RACING STARTED");
                } else {
                    static int last_second = -1;
                    int remaining = static_cast<int>(wait_duration_ - elapsed) + 1;
                    if (remaining != last_second) {
                        RCLCPP_INFO(get_logger(), "Starting in %d...", remaining);
                        last_second = remaining;
                    }
                }
                break;
            }

            case State::RACING: {
                std_msgs::msg::Bool msg;
                msg.data = true;
                lane_follow_pub_->publish(msg);
                
                double dist = distance_from_start();
                
                if (!has_left_start_ && has_left_start_area()) {
                    has_left_start_ = true;
                }
                
                if (has_left_start_ && is_in_start_box()) {
                    lap_count_++;
                    has_left_start_ = false;
                    RCLCPP_INFO(get_logger(), "=== RACING LAP %d COMPLETED! ===", lap_count_);
                }
                break;
            }

            case State::COMPLETED:
                break;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
