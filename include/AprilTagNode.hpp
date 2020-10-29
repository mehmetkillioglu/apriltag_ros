#pragma once

// ros
#include <rclcpp/rclcpp.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <image_transport/camera_subscriber.h>

// apriltag
#include <apriltag.h>
#include <apriltag_pose.h>

#include <Eigen/Core>


class AprilTagNode : public rclcpp::Node {
public:
    AprilTagNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions());

    ~AprilTagNode() override;

private:
    typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Mat3;

    apriltag_family_t* tf;
    apriltag_detector_t* const td;
    const std::string tag_family;
    const double tag_edge_size;
    const int max_hamming;
    std::unordered_map<int, std::string> tag_frames;
    std::unordered_map<int, double> tag_sizes;
    int process_rate;
    int cnt_;

    Mat3 K;
    sensor_msgs::msg::Image::ConstSharedPtr image_copy;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info_copy;


    const bool z_up;
    rclcpp::TimerBase::SharedPtr timer_;

    // function pointer for tag family creation / destruction
    static const std::map<std::string, apriltag_family_t *(*)(void)> tag_create;
    const static std::map<std::string, void (*)(apriltag_family_t*)> tag_destroy;

    const image_transport::CameraSubscriber sub_cam;
    const rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf;
    const rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;

    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);

    void getPose(const matd_t& H, geometry_msgs::msg::Transform& t, const double size) const;
    void detectApriltag();
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);
};
