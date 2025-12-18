#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"



class MissionManager: public rclcpp::Node 
{
    public:
        enum class State {
            IDLE,
            FORMATION_LAP,
            RACING,
            COMPLETED
        };

        MissionManager(): Node("mission_manager"){
            lap_count_ = -1; 
            is_in_box_ = false;
            was_in_box_ = false;
            is_start_recorded_ = false;
            current_state_ = State::IDLE;
            box_tolerance_ = 1.0;


            lane_follow_state_ = create_publisher<std_msgs::msg::Bool>(
                "/control/lane_follow", 10);

            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MissionManager::odom_callback, this, std::placeholders::_1));

            start_timer_ = create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&MissionManager::mission_control, this));
            RCLCPP_INFO(this->get_logger(), "Mission Manager Started!");
        }
 

    private:
        State current_state_;

        bool is_in_box_;
        bool was_in_box_;
        bool has_left_box_;

        int lap_count_;

        // Start Line
        geometry_msgs::msg::Point start_pose_;
        bool is_start_recorded_;
        double box_tolerance_;


        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lane_follow_state_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::TimerBase::SharedPtr start_timer_;

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom){
            if (!is_start_recorded_)
            {
                start_pose_ = odom->pose.pose.position;
                is_start_recorded_ = true;
            }

            double current_x = odom->pose.pose.position.x;
            double current_y = odom->pose.pose.position.y;

            bool inside_x = std::abs(current_x - start_pose_.x) < box_tolerance_;
            bool inside_y = std::abs(current_y - start_pose_.y) < box_tolerance_;

            is_in_box_ = (inside_x && inside_y);
        }

        void mission_control(){

            if (!is_in_box_) {
                has_left_box_ = true;
            }

            if (is_in_box_ && !was_in_box_ && has_left_box_) {
                
                if (current_state_ != State::IDLE && current_state_ != State::COMPLETED) {
                    lap_count_++;
                    has_left_box_ = false; 
                }
            }

            was_in_box_ = is_in_box_;
            
            switch (current_state_) {
                case State::IDLE:
                {
                     if(is_start_recorded_){
                        current_state_= State::FORMATION_LAP;
                        RCLCPP_INFO(get_logger(),"Formation Lap Started!");
                    }
                }
                break;

                case State::FORMATION_LAP:
                {

                    std_msgs::msg::Bool lane_msg;
                    lane_msg.data = true;
                    lane_follow_state_->publish(lane_msg);

                    if (lap_count_ >= 1) {
                        current_state_= State::RACING;
                        RCLCPP_INFO(get_logger(),"Racing Started!");
                    }
                }
                break;

                case State::RACING:
                break;

                case State::COMPLETED:
                break;
            }
        }
};


int main(int argc, char **argv){

    rclcpp::init(argc,argv);

    auto node = std::make_shared<MissionManager>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}