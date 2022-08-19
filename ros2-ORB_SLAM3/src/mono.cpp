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

class MonoSlamNode : public rclcpp::Node {
public:
  MonoSlamNode(const std::string &vocabFile, const std::string &settingsFile,
               const bool visualize);

  ~MonoSlamNode();

private:
  using ImageMsg = sensor_msgs::msg::Image;

  void GrabFrame(const sensor_msgs::msg::Image::SharedPtr msgRGB);

  std::shared_ptr<ORB_SLAM3::System> slam;

  rclcpp::Subscription<ImageMsg>::SharedPtr img_sub;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  bool visualize = !strcmp(argv[3], "true");
  auto node = std::make_shared<MonoSlamNode>(argv[1], argv[2], visualize);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

MonoSlamNode::MonoSlamNode(const std::string &vocabFile,
                           const std::string &settingsFile,
                           const bool visualize)
    : Node("orbslam3") {
  slam = std::make_shared<ORB_SLAM3::System>(
      vocabFile, settingsFile, ORB_SLAM3::System::MONOCULAR, visualize);
  img_sub = this->create_subscription<ImageMsg>(
      "orbslam3/camera", 10,
      std::bind(&MonoSlamNode::GrabFrame, this, std::placeholders::_1));

  setup_ros_publishers(*this);
}

MonoSlamNode::~MonoSlamNode() {
  if (!slam->isShutDown()) {
    // Stop all threads
    slam->Shutdown();

    // Save camera trajectory
    // slam->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  }
}

void MonoSlamNode::GrabFrame(const ImageMsg::SharedPtr msgImg) {
  const cv_bridge::CvImageConstPtr cv_ptrImg = cv_bridge::toCvShare(msgImg);

  cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(
      slam->TrackMonocular(cv_ptrImg->image, cv_ptrImg->header.stamp.sec)
          .matrix());

  rclcpp::Time current_frame_time = cv_ptrImg->header.stamp;

  publish_ros_pose_tf(*this, Tcw, current_frame_time,
                      ORB_SLAM3::System::MONOCULAR);

  publish_ros_tracking_mappoints(slam->GetTrackedMapPoints(),
                                 current_frame_time);

  publish_ros_tracking_img(
      ORB_SLAM3::Converter::toCvMat(
          ORB_SLAM3::Converter::toSE3Quat(slam->GetCurrentFrame().GetPose())),
      current_frame_time);
}
