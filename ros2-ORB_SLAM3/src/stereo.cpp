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
    using SyncMsg = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr &msgRGB,
                  const sensor_msgs::msg::Image::SharedPtr &msgD);

    void GrabStereo(const sensor_msgs::msg::Image::SharedPtr &msgLeft,
                    const sensor_msgs::msg::Image::SharedPtr &msgRight);

    void GrabMono(sensor_msgs::msg::Image::SharedPtr msgImg);

    ORB_SLAM3::System *slam;

    std::shared_ptr<message_filters::Subscriber<ImageMsg>> img_sub;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> left_sub;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> right_sub;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> rgb_sub;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> depth_sub;
    std::shared_ptr<message_filters::Synchronizer<SyncMsg>> syncApproximate;
};

int main(int argc, char **argv) {
    std::cerr << "STARTING" << std::endl;

    if (argc < 5) {
        std::cerr
                << "\nUsage: ros2 run ros2_orbslam3 <rgbd/monocular/stereo> </usr/src/ORB_SLAM3/Vocabulary/ORBvoc.txt> /usr/src/ORB_SLAM3/Examples/RGB-D/RealSense_D435i.yaml true"
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

    ORB_SLAM3::System* slam = new ORB_SLAM3::System(argv[2], argv[3], sensor_type, visualization);
    auto node = std::make_shared<SlamNode>(slam, sensor_type);
    rclcpp::spin(node);

    std::cerr << "POST ALL" << std::endl;

    rclcpp::shutdown();
    delete slam;


    return 0;
}

SlamNode::SlamNode(ORB_SLAM3::System *slam, const ORB_SLAM3::System::eSensor sensor)
        : Node("orbslam3"), slam(slam) {

    std::cerr << "preall" << std::endl;

    if (sensor == ORB_SLAM3::System::RGBD) {
        rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(shared_ptr<rclcpp::Node>(this), "camera/rgb");
        depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(shared_ptr<rclcpp::Node>(this), "camera/depth");
        syncApproximate = std::make_shared<message_filters::Synchronizer<SyncMsg>>(SyncMsg(10), *rgb_sub, *depth_sub);
        syncApproximate->registerCallback(&SlamNode::GrabRGBD, this);

    } else if (sensor == ORB_SLAM3::System::MONOCULAR) {
        img_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(shared_ptr<rclcpp::Node>(this), "camera");
        img_sub->registerCallback(&SlamNode::GrabMono, this);
    } else if (sensor == ORB_SLAM3::System::STEREO) {
        left_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(shared_ptr<rclcpp::Node>(this), "camera/left");
        right_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(shared_ptr<rclcpp::Node>(this), "camera/right");

        syncApproximate = std::make_shared<message_filters::Synchronizer<SyncMsg>>(SyncMsg(10), *left_sub, *right_sub);
        syncApproximate->registerCallback(&SlamNode::GrabStereo, this);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported sensor type");
    }

    std::cerr << "precall" << std::endl;
    setup_ros_publishers(*this);
    std::cerr << "postcall" << std::endl;
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
