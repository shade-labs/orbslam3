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

class RgbdSlamNode : public rclcpp::Node {
public:
  RgbdSlamNode(const std::string &vocabFile, const std::string &settingsFile,
               const bool visualize);

  ~RgbdSlamNode();

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using SyncMsg =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                      sensor_msgs::msg::Image>;

  void GrabFrame(const sensor_msgs::msg::Image::SharedPtr &msgRGB,
                 const sensor_msgs::msg::Image::SharedPtr &msgD);

  std::shared_ptr<ORB_SLAM3::System> slam;

  std::shared_ptr<message_filters::Subscriber<ImageMsg>> rgb_sub;
  std::shared_ptr<message_filters::Subscriber<ImageMsg>> depth_sub;
  std::shared_ptr<message_filters::Synchronizer<SyncMsg>> syncApproximate;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  bool visualize = !strcmp(argv[3], "true");
  auto node = std::make_shared<RgbdSlamNode>(argv[1], argv[2], visualize);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

RgbdSlamNode::RgbdSlamNode(const std::string &vocabFile,
                           const std::string &settingsFile,
                           const bool visualize)
    : Node("orbslam3") {

  slam = std::make_shared<ORB_SLAM3::System>(
      vocabFile, settingsFile, ORB_SLAM3::System::RGBD, visualize);
  rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(
      shared_ptr<rclcpp::Node>(this), "camera/rgb");
  depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(
      shared_ptr<rclcpp::Node>(this), "camera/depth");
  syncApproximate = std::make_shared<message_filters::Synchronizer<SyncMsg>>(
      SyncMsg(10), *rgb_sub, *depth_sub);
  syncApproximate->registerCallback(&RgbdSlamNode::GrabFrame, this);

  setup_ros_publishers(*this);
}

RgbdSlamNode::~RgbdSlamNode() {
  if (!slam->isShutDown()) {
    // Stop all threads
    slam->Shutdown();

    // Save camera trajectory
    // slam->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  }
}

void RgbdSlamNode::GrabFrame(const ImageMsg::SharedPtr &msgRGB,
                             const ImageMsg::SharedPtr &msgD) {
  const cv_bridge::CvImageConstPtr cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  const cv_bridge::CvImageConstPtr cv_ptrD = cv_bridge::toCvShare(msgD);

  cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(
      slam->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image,
                      cv_ptrRGB->header.stamp.sec)
          .matrix());

  rclcpp::Time current_frame_time = cv_ptrRGB->header.stamp;

  publish_ros_pose_tf(*this, Tcw, current_frame_time, ORB_SLAM3::System::RGBD);

  publish_ros_tracking_mappoints(slam->GetTrackedMapPoints(),
                                 current_frame_time);

  publish_ros_tracking_img(
      ORB_SLAM3::Converter::toCvMat(
          ORB_SLAM3::Converter::toSE3Quat(slam->GetCurrentFrame().GetPose())),
      current_frame_time);
}
