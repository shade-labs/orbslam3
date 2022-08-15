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
    SlamNode(ORB_SLAM3::System *slam);

    ~SlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using SyncMsg = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    void GrabFrame(sensor_msgs::msg::Image::SharedPtr msgImg);

    ORB_SLAM3::System *slam;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    bool visualization = !strcmp(argv[3], "true");
    ORB_SLAM3::System slam(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, visualization);
    SlamNode::SharedPtr node = std::make_shared<SlamNode>(&slam);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

SlamNode::SlamNode(ORB_SLAM3::System *slam)
        : Node("orbslam3_mono"), slam(slam) {
    img_sub = this->create_subscription<ImageMsg>("/camera", 10, std::bind(&SlamNode::GrabFrame, this, std::placeholders::_1));

    setup_ros_publishers(*this);
}

SlamNode::~SlamNode() {
    // Stop all threads
    slam->Shutdown();

    // Save camera trajectory
    slam->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void SlamNode::GrabFrame(const ImageMsg::SharedPtr msgImg) {
    const cv_bridge::CvImageConstPtr cv_ptrImg = cv_bridge::toCvShare(msgImg);

    cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(
            slam->TrackMonocular(cv_ptrImg->image, msgImg->header.stamp.sec).matrix());

    rclcpp::Time current_frame_time = msgImg->header.stamp;

//  publish_ros_pose_tf(*this, Tcw, current_frame_time, ORB_SLAM3::System::MONO);

    publish_ros_tracking_mappoints(slam->GetTrackedMapPoints(), current_frame_time);

    publish_ros_tracking_img(
            ORB_SLAM3::Converter::toCvMat(ORB_SLAM3::Converter::toSE3Quat(slam->GetCurrentFrame().GetPose())),
            current_frame_time);
}
