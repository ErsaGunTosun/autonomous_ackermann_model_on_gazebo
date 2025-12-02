#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/highgui/highgui.hpp>


class LaneFollow: public rclcpp::Node
{
    public:
    LaneFollow(): Node("lane_follow")
    {
        rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();

        subscriber_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera", 
            qos_profile,     
            std::bind(&LaneFollow::image_callback, this, std::placeholders::_1));

        cv::namedWindow("CAMERA");
    }

    ~LaneFollow()
    {
        cv::destroyAllWindows();
    }

    private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr img)
    {
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

        cv::imshow("CAMERA", img_bgr);
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