#include "common.hpp"

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rclcpp/rclcpp.hpp>

// ORB-SLAM3-specific libraries. Directory is defined in CMakeLists.txt: ${ORB_SLAM3_DIR}
#include "System.h"
#include "ImuTypes.h"
#include "Converter.h"
#include "SerializationUtils.h"
#include "Tracking.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "Atlas.h"
#include "Settings.h"

using std::placeholders::_1;

class RgbdSlamNode : public rclcpp::Node {
public:
  RgbdSlamNode(ORB_SLAM3::System *m_SLAM);

  ~RgbdSlamNode();

private:
  using ImageMsg = sensor_msgs::msg::Image;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>
      approximate_sync_policy;

  void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr& msgRGB,
                const sensor_msgs::msg::Image::SharedPtr& msgD);

  ORB_SLAM3::System *m_SLAM;

  cv_bridge::CvImageConstPtr cv_ptrRGB;
  cv_bridge::CvImageConstPtr cv_ptrD;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>
      depth_sub;

  std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>>
      syncApproximate;
};

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr
        << "\nUsage: ros2 run orbslam rgbd path_to_vocabulary path_to_settings"
        << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);

  // malloc error using new.. try shared ptr
  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.

  bool visualization = false;
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, visualization);

  auto node = std::make_shared<RgbdSlamNode>(&SLAM);
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System *m_SLAM)
    : Node("orbslam"), m_SLAM(m_SLAM) {
  rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>( shared_ptr<rclcpp::Node>(this), "orbslam3/image_raw");
  depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>( shared_ptr<rclcpp::Node>(this), "orbslam3/depth_depth");

  syncApproximate =
      std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
          approximate_sync_policy(10), *rgb_sub, *depth_sub);
  syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

  setup_ros_publishers(*this);
}

RgbdSlamNode::~RgbdSlamNode() {
  // Stop all threads
  m_SLAM->Shutdown();

  // Save camera trajectory
  m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr& msgRGB,
                            const ImageMsg::SharedPtr& msgD) {
  cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  cv_ptrD = cv_bridge::toCvShare(msgD);

  cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.sec).matrix());

  rclcpp::Time current_frame_time = cv_ptrRGB->header.stamp;

//  publish_ros_pose_tf(*this, Tcw, current_frame_time, ORB_SLAM3::System::STEREO);

  publish_ros_tracking_mappoints(m_SLAM->GetTrackedMapPoints(), current_frame_time);

//  publish_ros_tracking_img(ORB_SLAM3::Converter::toCvMat(ORB_SLAM3::Converter::toSE3Quat(m_SLAM->GetCurrentFrame().GetPose())), current_frame_time);
}
