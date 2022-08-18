#include "common.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

// ORB-SLAM3-specific libraries. Directory is defined in CMakeLists.txt:
// ${ORB_SLAM3_DIR}
#include "Converter.h"
#include "System.h"
#include "Tracking.h"

using std::placeholders::_1;

class StereoSlamNode : public rclcpp::Node {
public:
  StereoSlamNode(const std::string &vocabFile, const std::string &settingsFile,
                 const bool visualize);

  ~StereoSlamNode();

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using SyncMsg =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                      sensor_msgs::msg::Image>;

  void GrabFrame(const sensor_msgs::msg::Image::SharedPtr &msgRight,
                 const sensor_msgs::msg::Image::SharedPtr &msgLeft);

  std::shared_ptr<ORB_SLAM3::System> slam;

  std::shared_ptr<message_filters::Subscriber<ImageMsg>> right_sub;
  std::shared_ptr<message_filters::Subscriber<ImageMsg>> left_sub;
  std::shared_ptr<message_filters::Synchronizer<SyncMsg>> syncApproximate;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  bool visualize = !strcmp(argv[3], "true");
  auto node = std::make_shared<StereoSlamNode>(argv[1], argv[2], visualize);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

StereoSlamNode::StereoSlamNode(const std::string &vocabFile,
                               const std::string &settingsFile,
                               const bool visualize)
    : Node("orbslam3") {

  slam = std::make_shared<ORB_SLAM3::System>(
      vocabFile, settingsFile, ORB_SLAM3::System::STEREO, visualize);
  right_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(
      shared_ptr<rclcpp::Node>(this), "orbslam3/camera/right");
  left_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(
      shared_ptr<rclcpp::Node>(this), "orbslam3/camera/left");
  syncApproximate = std::make_shared<message_filters::Synchronizer<SyncMsg>>(
      SyncMsg(10), *right_sub, *left_sub);
  syncApproximate->registerCallback(&StereoSlamNode::GrabFrame, this);

  setup_ros_publishers(*this);
}

StereoSlamNode::~StereoSlamNode() {
  if (!slam->isShutDown()) {
    // Stop all threads
    slam->Shutdown();

    // Save camera trajectory
    // slam->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  }
}

void StereoSlamNode::GrabFrame(const ImageMsg::SharedPtr &msgRight,
                               const ImageMsg::SharedPtr &msgLeft) {
  const cv_bridge::CvImageConstPtr cv_ptrRight = cv_bridge::toCvShare(msgRight);
  const cv_bridge::CvImageConstPtr cv_ptrLeft = cv_bridge::toCvShare(msgLeft);

  cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(
      slam->TrackStereo(cv_ptrRight->image, cv_ptrRight->image,
                        cv_ptrRight->header.stamp.sec)
          .matrix());

  rclcpp::Time current_frame_time = cv_ptrRight->header.stamp;

  publish_ros_pose_tf(*this, Tcw, current_frame_time,
                      ORB_SLAM3::System::STEREO);

  publish_ros_tracking_mappoints(slam->GetTrackedMapPoints(),
                                 current_frame_time);

  publish_ros_tracking_img(
      ORB_SLAM3::Converter::toCvMat(
          ORB_SLAM3::Converter::toSE3Quat(slam->GetCurrentFrame().GetPose())),
      current_frame_time);
}
