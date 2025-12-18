#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp" 

using std::placeholders::_1;

class LaneController : public rclcpp::Node
{
public:
  LaneController() : Node("lane_controller")
  {
    lane_follow_state_ = create_subscription<std_msgs::msg::Bool>(
            "/control/lane_follow", 
            100,     
            std::bind(&LaneController::lane_follow_callback, this, std::placeholders::_1));

    subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "/lane/error", 10, std::bind(&LaneController::topic_callback, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    kp_ = 0.005; 
    base_speed_ = 0.5; 
  }

private:

  void lane_follow_callback(const std_msgs::msg::Bool::SharedPtr msg){
        bool data = msg->data;
        if (is_start && !data){
            is_start = false;
        }
        else if (!is_start && data){
            is_start = true;
        }
  }

  void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) const
  {
    if(!is_start)return;

    float error = msg->data;
    float steering_angle = -1.0 * kp_ * error; 

    if (steering_angle > 0.5) steering_angle = 0.5;
    if (steering_angle < -0.5) steering_angle = -0.5;

    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = base_speed_;
    cmd_msg.angular.z = steering_angle;

    publisher_->publish(cmd_msg);
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lane_follow_state_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  
  double kp_;
  double base_speed_;
  bool is_start;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneController>());
  rclcpp::shutdown();
  return 0;
}