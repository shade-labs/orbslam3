#include "common.hpp"

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <memory>
#include <chrono>
#include <mutex>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ORB-SLAM3-specific libraries. Directory is defined in CMakeLists.txt: ${ORB_SLAM3_DIR}
#include "System.h"
#include "Converter.h"
#include "Tracking.h"

using std::placeholders::_1;

class SlamNode : public rclcpp::Node {
public:
    SlamNode(ORB_SLAM3::System *slam, const ORB_SLAM3::System::eSensor sensor);

    ~SlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image>
            approximate_sync_policy;

    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr &msgRGB,
                  const sensor_msgs::msg::Image::SharedPtr &msgD);

    void GrabStereo(const sensor_msgs::msg::Image::SharedPtr &msgLeft,
                    const sensor_msgs::msg::Image::SharedPtr &msgRight);

    void GrabMono(sensor_msgs::msg::Image::SharedPtr msgImg);

    ORB_SLAM3::System *slam;
};

int main(int argc, char **argv) {
    if (argc < 5) {
        std::cerr
                << "\nUsage: ros2 run ros2_orbslam3 slam sensor_type path_to_vocabulary path_to_settings visualize"
                << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    bool visualization = !strcmp(argv[4], "true");
    std::string sensor(argv[1]);

    ORB_SLAM3::System::eSensor sensor_type;
    if (sensor == "rgbd") {
        sensor_type = ORB_SLAM3::System::RGBD;
    } else if (sensor == "monocular") {
        sensor_type = ORB_SLAM3::System::MONOCULAR;
    } else if (sensor == "stereo") {
        sensor_type = ORB_SLAM3::System::STEREO;
    } else {
        cerr << "Invalid sensor " << sensor << endl;
        return 1;
    }

    ORB_SLAM3::System slam(argv[2], argv[3], sensor_type, visualization);
    auto node = std::make_shared<SlamNode>(&slam, sensor_type);
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

SlamNode::SlamNode(ORB_SLAM3::System *slam, const ORB_SLAM3::System::eSensor sensor)
        : Node("orbslam"), slam(slam) {

    if (sensor == ORB_SLAM3::System::RGBD) {
        std::shared_ptr<message_filters::Subscriber<ImageMsg>> rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(
                shared_ptr<rclcpp::Node>(this),
                "camera/rgb");
        std::shared_ptr<message_filters::Subscriber<ImageMsg>> depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(
                shared_ptr<rclcpp::Node>(this),
                "camera/depth");

        std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate =
                std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
                        approximate_sync_policy(10), *rgb_sub, *depth_sub);
        syncApproximate->registerCallback(&SlamNode::GrabRGBD, this);

    } else if (sensor == ORB_SLAM3::System::MONOCULAR) {
        rclcpp::Subscription<ImageMsg>::SharedPtr img_sub = this->create_subscription<sensor_msgs::msg::Image>
                ("camera", 10, std::bind(&SlamNode::GrabMono, this, _1));

    } else if (sensor == ORB_SLAM3::System::STEREO) {
        std::shared_ptr<message_filters::Subscriber<ImageMsg>> left_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(
                shared_ptr<rclcpp::Node>(this),
                "camera/left");
        std::shared_ptr<message_filters::Subscriber<ImageMsg>> right_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(
                shared_ptr<rclcpp::Node>(this),
                "camera/right");

        std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(
                approximate_sync_policy(10), *left_sub, *right_sub);
        syncApproximate->registerCallback(&SlamNode::GrabStereo, this);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported sensor type");
    }

    setup_ros_publishers(*this);
}

SlamNode::~SlamNode() {
    // Stop all threads
    slam->Shutdown();

    // Save camera trajectory
    slam->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void SlamNode::GrabRGBD(const ImageMsg::SharedPtr &msgRGB,
                        const ImageMsg::SharedPtr &msgD) {
    const cv_bridge::CvImageConstPtr cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    const cv_bridge::CvImageConstPtr cv_ptrD = cv_bridge::toCvShare(msgD);

    cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(
            slam->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.sec).matrix());

    rclcpp::Time current_frame_time = cv_ptrRGB->header.stamp;

//  publish_ros_pose_tf(*this, Tcw, current_frame_time, ORB_SLAM3::System::STEREO);

    publish_ros_tracking_mappoints(slam->GetTrackedMapPoints(), current_frame_time);

    publish_ros_tracking_img(
            ORB_SLAM3::Converter::toCvMat(ORB_SLAM3::Converter::toSE3Quat(slam->GetCurrentFrame().GetPose())),
            current_frame_time);
}

void SlamNode::GrabStereo(const ImageMsg::SharedPtr &msgLeft,
                          const ImageMsg::SharedPtr &msgRight) {
    const cv_bridge::CvImageConstPtr cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    const cv_bridge::CvImageConstPtr cv_ptrRight = cv_bridge::toCvShare(msgRight);

    cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(
            slam->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrRight->header.stamp.sec).matrix());

    rclcpp::Time current_frame_time = cv_ptrLeft->header.stamp;

//  publish_ros_pose_tf(*this, Tcw, current_frame_time, ORB_SLAM3::System::STEREO);

    publish_ros_tracking_mappoints(slam->GetTrackedMapPoints(), current_frame_time);

    publish_ros_tracking_img(
            ORB_SLAM3::Converter::toCvMat(ORB_SLAM3::Converter::toSE3Quat(slam->GetCurrentFrame().GetPose())),
            current_frame_time);
}

void SlamNode::GrabMono(const ImageMsg::SharedPtr msgImg) {
    const cv_bridge::CvImageConstPtr cv_ptrImg = cv_bridge::toCvShare(msgImg);

    cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(
            slam->TrackMonocular(cv_ptrImg->image, cv_ptrImg->header.stamp.sec).matrix());

    rclcpp::Time current_frame_time = cv_ptrImg->header.stamp;

//  publish_ros_pose_tf(*this, Tcw, current_frame_time, ORB_SLAM3::System::STEREO);

    publish_ros_tracking_mappoints(slam->GetTrackedMapPoints(), current_frame_time);

    publish_ros_tracking_img(
            ORB_SLAM3::Converter::toCvMat(ORB_SLAM3::Converter::toSE3Quat(slam->GetCurrentFrame().GetPose())),
            current_frame_time);
}
