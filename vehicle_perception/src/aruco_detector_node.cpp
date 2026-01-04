#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cv_bridge/cv_bridge.h"

#include <vector>
#include <cmath>

class ArucoDetector : public rclcpp::Node
{
public:
    ArucoDetector() : Node("aruco_detector")
    {
        this->declare_parameter<double>("marker_size", 0.15); 
        this->declare_parameter<bool>("show_debug_window", true);
        this->declare_parameter<int>("dictionary_id", 0);  
        
        marker_size_ = this->get_parameter("marker_size").as_double();
        show_debug_ = this->get_parameter("show_debug_window").as_bool();
        int dict_id = this->get_parameter("dictionary_id").as_int();
        
        dictionary_ = cv::aruco::getPredefinedDictionary(dict_id);
        detector_params_ = cv::aruco::DetectorParameters::create();
        
        detector_params_->adaptiveThreshWinSizeMin = 3;
        detector_params_->adaptiveThreshWinSizeMax = 23;
        detector_params_->adaptiveThreshWinSizeStep = 10;
        detector_params_->minMarkerPerimeterRate = 0.03;
        detector_params_->maxMarkerPerimeterRate = 4.0;
        detector_params_->polygonalApproxAccuracyRate = 0.03;
        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        
        double fx = 554.254; 
        double fy = 554.254;  
        double cx = 320.0;    
        double cy = 240.0;    
        
        camera_matrix_ = (cv::Mat_<double>(3, 3) << 
            fx, 0, cx,
            0, fy, cy,
            0, 0, 1);
        
        dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
        
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera",
            rclcpp::SensorDataQoS(),
            std::bind(&ArucoDetector::image_callback, this, std::placeholders::_1));
        
        marker_ids_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/aruco/detected_ids", 10);
        
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/aruco/pose", 10);
        
        pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/aruco/poses", 10);
        
        vis_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/aruco/markers", 10);
        
        if (show_debug_) {
            cv::namedWindow("ArUco Detection", cv::WINDOW_AUTOSIZE);
        }
        
        RCLCPP_INFO(this->get_logger(), "ArUco Detector started. Marker size: %.3f m", marker_size_);
    }
    
    ~ArucoDetector()
    {
        if (show_debug_) {
            cv::destroyWindow("ArUco Detection");
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr marker_ids_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_marker_pub_;
    
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    
    double marker_size_;
    bool show_debug_;
    
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }
        
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty image received!");
            return;
        }
        
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
        
        cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids, 
                                  detector_params_, rejected_candidates);
        
        auto ids_msg = std_msgs::msg::Int32MultiArray();
        ids_msg.data = marker_ids;
        marker_ids_pub_->publish(ids_msg);
        
        auto pose_array_msg = geometry_msgs::msg::PoseArray();
        pose_array_msg.header.stamp = msg->header.stamp;
        pose_array_msg.header.frame_id = "camera_link";
        
        auto vis_markers = visualization_msgs::msg::MarkerArray();
        
        if (!marker_ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, 
                                                  camera_matrix_, dist_coeffs_, 
                                                  rvecs, tvecs);
            
            double min_distance = std::numeric_limits<double>::max();
            int closest_idx = -1;
            
            for (size_t i = 0; i < marker_ids.size(); i++) {
                double distance = cv::norm(tvecs[i]);
                
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_idx = static_cast<int>(i);
                }
                
                geometry_msgs::msg::Pose pose;
                pose.position.x = tvecs[i][0];
                pose.position.y = tvecs[i][1];
                pose.position.z = tvecs[i][2];
                
                cv::Mat rotation_matrix;
                cv::Rodrigues(rvecs[i], rotation_matrix);
                
                double trace = rotation_matrix.at<double>(0, 0) + 
                               rotation_matrix.at<double>(1, 1) + 
                               rotation_matrix.at<double>(2, 2);
                double qw, qx, qy, qz;
                
                if (trace > 0) {
                    double s = 0.5 / std::sqrt(trace + 1.0);
                    qw = 0.25 / s;
                    qx = (rotation_matrix.at<double>(2, 1) - rotation_matrix.at<double>(1, 2)) * s;
                    qy = (rotation_matrix.at<double>(0, 2) - rotation_matrix.at<double>(2, 0)) * s;
                    qz = (rotation_matrix.at<double>(1, 0) - rotation_matrix.at<double>(0, 1)) * s;
                } else {
                    if (rotation_matrix.at<double>(0, 0) > rotation_matrix.at<double>(1, 1) && 
                        rotation_matrix.at<double>(0, 0) > rotation_matrix.at<double>(2, 2)) {
                        double s = 2.0 * std::sqrt(1.0 + rotation_matrix.at<double>(0, 0) - 
                                                   rotation_matrix.at<double>(1, 1) - 
                                                   rotation_matrix.at<double>(2, 2));
                        qw = (rotation_matrix.at<double>(2, 1) - rotation_matrix.at<double>(1, 2)) / s;
                        qx = 0.25 * s;
                        qy = (rotation_matrix.at<double>(0, 1) + rotation_matrix.at<double>(1, 0)) / s;
                        qz = (rotation_matrix.at<double>(0, 2) + rotation_matrix.at<double>(2, 0)) / s;
                    } else if (rotation_matrix.at<double>(1, 1) > rotation_matrix.at<double>(2, 2)) {
                        double s = 2.0 * std::sqrt(1.0 + rotation_matrix.at<double>(1, 1) - 
                                                   rotation_matrix.at<double>(0, 0) - 
                                                   rotation_matrix.at<double>(2, 2));
                        qw = (rotation_matrix.at<double>(0, 2) - rotation_matrix.at<double>(2, 0)) / s;
                        qx = (rotation_matrix.at<double>(0, 1) + rotation_matrix.at<double>(1, 0)) / s;
                        qy = 0.25 * s;
                        qz = (rotation_matrix.at<double>(1, 2) + rotation_matrix.at<double>(2, 1)) / s;
                    } else {
                        double s = 2.0 * std::sqrt(1.0 + rotation_matrix.at<double>(2, 2) - 
                                                   rotation_matrix.at<double>(0, 0) - 
                                                   rotation_matrix.at<double>(1, 1));
                        qw = (rotation_matrix.at<double>(1, 0) - rotation_matrix.at<double>(0, 1)) / s;
                        qx = (rotation_matrix.at<double>(0, 2) + rotation_matrix.at<double>(2, 0)) / s;
                        qy = (rotation_matrix.at<double>(1, 2) + rotation_matrix.at<double>(2, 1)) / s;
                        qz = 0.25 * s;
                    }
                }
                
                pose.orientation.x = qx;
                pose.orientation.y = qy;
                pose.orientation.z = qz;
                pose.orientation.w = qw;
                
                pose_array_msg.poses.push_back(pose);
                
                visualization_msgs::msg::Marker vis_marker;
                vis_marker.header.stamp = msg->header.stamp;
                vis_marker.header.frame_id = "camera_link";
                vis_marker.ns = "aruco_markers";
                vis_marker.id = marker_ids[i];
                vis_marker.type = visualization_msgs::msg::Marker::CUBE;
                vis_marker.action = visualization_msgs::msg::Marker::ADD;
                vis_marker.pose = pose;
                vis_marker.scale.x = marker_size_;
                vis_marker.scale.y = marker_size_;
                vis_marker.scale.z = 0.01;
                vis_marker.color.r = 0.0;
                vis_marker.color.g = 1.0;
                vis_marker.color.b = 0.0;
                vis_marker.color.a = 0.8;
                vis_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
                
                vis_markers.markers.push_back(vis_marker);
                
                if (show_debug_) {
                    cv::aruco::drawAxis(frame, camera_matrix_, dist_coeffs_, 
                                        rvecs[i], tvecs[i], marker_size_ * 0.5);
                }
                
                RCLCPP_DEBUG(this->get_logger(), 
                    "Marker ID: %d, Mesafe: %.2f m, Pozisyon: (%.2f, %.2f, %.2f)",
                    marker_ids[i], distance, tvecs[i][0], tvecs[i][1], tvecs[i][2]);
            }
            
            if (closest_idx >= 0) {
                auto pose_msg = geometry_msgs::msg::PoseStamped();
                pose_msg.header.stamp = msg->header.stamp;
                pose_msg.header.frame_id = "camera_link";
                pose_msg.pose = pose_array_msg.poses[closest_idx];
                pose_pub_->publish(pose_msg);
                
                static int last_logged_marker = -1;
                if (marker_ids[closest_idx] != last_logged_marker) {
                    RCLCPP_INFO(this->get_logger(),
                        "ArUco marker %d detected (%.2f m)", 
                        marker_ids[closest_idx], min_distance);
                    last_logged_marker = marker_ids[closest_idx];
                } else {
                    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "Marker %d: %.2f m", marker_ids[closest_idx], min_distance);
                }
            }
        }
        
        pose_array_pub_->publish(pose_array_msg);
        vis_marker_pub_->publish(vis_markers);
        
        if (show_debug_) {
            if (!marker_ids.empty()) {
                cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);
            }
            
            std::string info_text = "Detected markers: " + std::to_string(marker_ids.size());
            cv::putText(frame, info_text, cv::Point(10, 30), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            
            cv::imshow("ArUco Detection", frame);
            cv::waitKey(1);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
