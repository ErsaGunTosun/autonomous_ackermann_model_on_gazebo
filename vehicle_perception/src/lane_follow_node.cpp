#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cv_bridge/cv_bridge.h"

class LaneFollow: public rclcpp::Node
{
    public:
    LaneFollow(): Node("lane_follow")
    {
         lane_follow_state_ = create_subscription<std_msgs::msg::Bool>(
            "/control/lane_follow", 
            100,     
            std::bind(&LaneFollow::lane_follow_callback, this, std::placeholders::_1));

        subscriber_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera", 
            rclcpp::SensorDataQoS(),  
            std::bind(&LaneFollow::image_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/lane/error", 10);
        cv::namedWindow("CAMERA");
    }

    ~LaneFollow()
    {
        cv::destroyAllWindows();
    }

    private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lane_follow_state_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;


    const cv::Scalar LOWER_COLOR_1 = cv::Scalar(0, 100, 100);    
    const cv::Scalar UPPER_COLOR_1 = cv::Scalar(10, 255, 255); 
    const cv::Scalar LOWER_COLOR_2 = cv::Scalar(160, 100, 100); 
    const cv::Scalar UPPER_COLOR_2 = cv::Scalar(180, 255, 255); 

    const int ROI_HEIGHT_RATIO = 5; 


    bool is_start = false;

    void lane_follow_callback(const std_msgs::msg::Bool::SharedPtr msg){
        bool data = msg->data;
        if (is_start && !data){
            is_start = false;
        }
        else if (!is_start && data){
            is_start = true;
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr img)
    {
        if (!is_start) return;

        cv::Mat img_bgr;
        try
        {
            img_bgr = cv_bridge::toCvCopy(img, "bgr8")->image;
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge hatasi: %s", e.what());
            return;
        }

        if (img_bgr.empty()) {
            RCLCPP_WARN(get_logger(), "Bos goruntu geldi!");
            return;
        }

        int roi_start_row = img_bgr.rows / 2; 
        cv::Mat roi_img = img_bgr(cv::Range(roi_start_row, img_bgr.rows), cv::Range::all());

        cv::Mat hsv_image;
        cv::cvtColor(roi_img, hsv_image, cv::COLOR_BGR2HSV);

        cv::Mat mask_1, mask_2, mask; 
        cv::inRange(hsv_image, LOWER_COLOR_1, UPPER_COLOR_1, mask_1);
        cv::inRange(hsv_image, LOWER_COLOR_2, UPPER_COLOR_2, mask_2);
        cv::bitwise_or(mask_1, mask_2, mask);

        cv::bitwise_not(mask, mask);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::erode(mask, mask, kernel);
        cv::dilate(mask, mask, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double max_area = 0;
        int max_idx = -1;

        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > max_area) {
                max_area = area;
                max_idx = i;
            }
        }

        double err = 0.0;
        bool road_found = false;

        cv::Mat debug_view;
        cv::cvtColor(mask, debug_view, cv::COLOR_GRAY2BGR);

        if (max_idx != -1 && max_area > 1000) 
        {
            road_found = true;
            
            cv::Moments M = cv::moments(contours[max_idx]);
            if (M.m00 > 0) {
                int cx = int(M.m10 / M.m00);
                int cy = int(M.m01 / M.m00);

                int width = roi_img.cols;
                err = cx - (width / 2);

                auto msg = std_msgs::msg::Float32();
                msg.data = err;
                publisher_->publish(msg);

                cv::drawContours(debug_view, contours, max_idx, cv::Scalar(0, 255, 0), 2);
                cv::circle(debug_view, cv::Point(cx, cy), 10, cv::Scalar(0, 0, 255), -1);
            }
        }

        cv::imshow("CAMERA", debug_view);
        cv::waitKey(1);
    }

};


int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);

    auto node = std::make_shared<LaneFollow>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}