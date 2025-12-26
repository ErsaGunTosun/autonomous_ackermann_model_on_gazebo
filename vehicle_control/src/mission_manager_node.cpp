#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include <cmath>
#include <cstdlib>

class MissionManager : public rclcpp::Node 
{
public:
    enum class State {
        IDLE,
        FORMATION_LAP,
        WAITING_LOOP_CLOSURE,
        SAVING_MAP,
        STARTING_NAV2,
        RACING
    };

    MissionManager() : Node("mission_manager") {
        current_state_ = State::IDLE;
        lap_count_ = 0;
        
        start_box_radius_ = 1.5;
        min_leave_distance_ = 3.0;
        loop_closure_wait_time_ = 10.0;
    
        start_recorded_ = false;
        has_left_start_ = false;
        map_saved_ = false;
        nav2_started_ = false;
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        lane_follow_pub_ = create_publisher<std_msgs::msg::Bool>("/control/lane_follow", 10);
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        state_pub_ = create_publisher<std_msgs::msg::String>("/mission/state", 10);
        
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&MissionManager::mission_control, this));
        
        RCLCPP_INFO(get_logger(), "===========================================");
        RCLCPP_INFO(get_logger(), "   MISSION MANAGER STARTED");
        RCLCPP_INFO(get_logger(), "===========================================");
    }

private:
    State current_state_;
    int lap_count_;
    
    double start_box_radius_;
    double min_leave_distance_;
    double loop_closure_wait_time_;
    
    double start_x_, start_y_;
    double current_x_, current_y_;
    bool start_recorded_;
    bool has_left_start_;
    bool map_saved_;
    bool nav2_started_;
    
    rclcpp::Time state_start_time_;
    rclcpp::Time loop_closure_start_time_;
    rclcpp::Time nav2_start_time_;
    
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

    bool is_in_start_box() { return distance_from_start() < start_box_radius_; }
    bool has_left_start_area() { return distance_from_start() > min_leave_distance_; }

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
        std::system(
            "ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "
            "\"{name: {data: '/tmp/formation_map'}}\" &"
        );
        map_saved_ = true;
    }

    void start_nav2() {
        RCLCPP_INFO(get_logger(), "Starting Nav2...");
        std::system("ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=/home/ersagun/Projects/ackermann_ws/src/vehicle_navigation/config/nav2_params_ackermann.yaml autostart:=true &");
        nav2_started_ = true;
        nav2_start_time_ = this->now();
    }

    void mission_control() {
        if (!get_robot_position()) return;
        
        switch (current_state_) {
            case State::IDLE: {
                publish_state("IDLE");
                if (!start_recorded_) {
                    start_x_ = current_x_;
                    start_y_ = current_y_;
                    start_recorded_ = true;
                    has_left_start_ = false;
                    
                    RCLCPP_INFO(get_logger(), "FORMATION LAP STARTING - Start: (%.2f, %.2f)", start_x_, start_y_);
                    current_state_ = State::FORMATION_LAP;
                    state_start_time_ = this->now();
                }
                break;
            }

            case State::FORMATION_LAP: {
                publish_state("FORMATION_LAP");
                set_lane_follow(true);
                
                if (!has_left_start_ && has_left_start_area()) {
                    has_left_start_ = true;
                    RCLCPP_INFO(get_logger(), "Left start area!");
                }
                
                if (has_left_start_ && is_in_start_box()) {
                    lap_count_++;
                    RCLCPP_INFO(get_logger(), "FORMATION LAP COMPLETED!");
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
                    current_state_ = State::STARTING_NAV2;
                }
                break;
            }

            case State::STARTING_NAV2: {
                publish_state("STARTING_NAV2");
                stop_vehicle();
                
                if (!nav2_started_) {
                    start_nav2();
                }
                
                // Nav2 başladıktan 15 saniye sonra racing moduna geç
                double elapsed = (this->now() - nav2_start_time_).seconds();
                if (nav2_started_ && elapsed >= 15.0) {
                    RCLCPP_INFO(get_logger(), "Nav2 started. Switching to RACING mode.");
                    RCLCPP_INFO(get_logger(), "Use RViz 2D Goal Pose to send navigation goals!");
                    current_state_ = State::RACING;
                    has_left_start_ = false;
                }
                break;
            }

            case State::RACING: {
                publish_state("RACING");
                // Nav2 kontrolünde - RViz'den goal gönderilecek
                // Veya lane follow ile devam edilebilir
                set_lane_follow(false);  // Nav2 kontrol etsin
                break;
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionManager>());
    rclcpp::shutdown();
    return 0;
}
