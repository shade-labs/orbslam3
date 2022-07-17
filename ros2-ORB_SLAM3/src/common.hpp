#pragma once

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


extern std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_pub;
extern std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> map_points_pub;
extern image_transport::Publisher rendered_image_pub;

extern std::string map_frame_id, pose_frame_id;


void setup_ros_publishers(rclcpp::Node &node);

//void setup_tf_orb_to_ros(ORB_SLAM3::System::eSensor);
//
//
//void publish_ros_pose_tf(rclcpp::Node &node, cv::Mat, rclcpp::Time, ORB_SLAM3::System::eSensor);
//void publish_tf_transform(rclcpp::Node &node, tf2::Transform, rclcpp::Time);
//void publish_pose_stamped(tf2::Transform, rclcpp::Time);
void publish_ros_tracking_img(const cv::Mat &, const rclcpp::Time &);

void publish_ros_tracking_mappoints(std::vector<ORB_SLAM3::MapPoint *>, const rclcpp::Time &);


//tf2::Transform from_orb_to_ros_tf_transform(cv::Mat);
sensor_msgs::msg::PointCloud2 tracked_mappoints_to_pointcloud(std::vector<ORB_SLAM3::MapPoint *>, rclcpp::Time);

