#include "common.hpp"

//std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_pub;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>
    map_points_pub;
//image_transport::Publisher rendered_image_pub;

std::string map_frame_id, pose_frame_id;

// Coordinate transformation matrix from orb coordinate system to ros coordinate
// system
tf2::Matrix3x3 tf_orb_to_ros(1, 0, 0, 0, 1, 0, 0, 0, 1);

void setup_ros_publishers(rclcpp::Node &node) {
//  pose_pub = node.create_publisher<geometry_msgs::msg::PoseStamped>("/orbslam3/camera", 1);

  map_points_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("orbslam3/map_points", 1);

//  std::shared_ptr<rclcpp::Node> image_transport_node = rclcpp::Node::make_shared("image_publisher");
//  image_transport::ImageTransport image_transport(image_transport_node);
//
//  rendered_image_pub = image_transport.advertise("orbslam3/tracking_image", 1);
}

//void publish_ros_pose_tf(rclcpp::Node &node,
//                         cv::Mat Tcw, rclcpp::Time current_frame_time,
//                         ORB_SLAM3::System::eSensor sensor_type) {
//  if (!Tcw.empty()) {
//    tf2::Transform tf_transform =
//        from_orb_to_ros_tf_transform(Tcw);

//    publish_tf_transform(node, tf_transform, current_frame_time);

//    publish_pose_stamped(tf_transform, current_frame_time);
//  }
//}

//void publish_tf_transform(rclcpp::Node &node, tf2::Transform tf_transform,
//                          rclcpp::Time current_frame_time) {
//  static tf2_ros::TransformBroadcaster tf_broadcaster(node);
//
//  std_msgs::msg::Header header;
//  header.stamp = current_frame_time;
//  header.frame_id = map_frame_id;
//
//  geometry_msgs::msg::TransformStamped tf_msg;
//  tf_msg.header = header;
//  tf_msg.child_frame_id = pose_frame_id;
//  tf_msg.transform = tf2::toMsg(tf_transform);
//
//  tf_broadcaster.sendTransform(tf_msg);
//}

//void publish_pose_stamped(tf2::Transform tf_transform, rclcpp::Time current_frame_time) {
//  std_msgs::msg::Header header;
//  header.stamp = current_frame_time;
//  header.frame_id = map_frame_id;
//
//  geometry_msgs::msg::Pose pose;
//  tf2::toMsg(tf_transform, pose);
//
//  geometry_msgs::msg::PoseStamped pose_msg;
//  pose_msg.header = header;
//  pose_msg.pose = pose;
//
//  pose_pub->publish(pose_msg);
//}

//void publish_ros_tracking_img(cv::Mat image, rclcpp::Time current_frame_time) {
//  std_msgs::msg::Header header;
//  header.stamp = current_frame_time;
//  header.frame_id = map_frame_id;
//
//  const std::shared_ptr<sensor_msgs::msg::Image> rendered_image_msg =
//      cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
//
//  rendered_image_pub.publish(rendered_image_msg);
//}

void publish_ros_tracking_mappoints(
    std::vector<ORB_SLAM3::MapPoint *> map_points,
    rclcpp::Time current_frame_time) {
  sensor_msgs::msg::PointCloud2 cloud =
      tracked_mappoints_to_pointcloud(map_points, current_frame_time);

  map_points_pub->publish(cloud);
}

//void setup_tf_orb_to_ros(ORB_SLAM3::System::eSensor sensor_type) {
//  // The conversion depends on whether IMU is involved:
//  //  z is aligned with camera's z axis = without IMU
//  //  z is aligned with gravity = with IMU
//  if (sensor_type == ORB_SLAM3::System::MONOCULAR ||
//      sensor_type == ORB_SLAM3::System::STEREO ||
//      sensor_type == ORB_SLAM3::System::RGBD) {
//    tf_orb_to_ros.setValue(0, 0, 1, -1, 0, 0, 0, -1, 0);
//  } else if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR ||
//             sensor_type == ORB_SLAM3::System::IMU_STEREO) {
//    tf_orb_to_ros.setValue(0, 1, 0, -1, 0, 0, 0, 0, 1);
//  } else {
//    tf_orb_to_ros.setIdentity();
//  }
//}

//tf2::Transform
//from_orb_to_ros_tf_transform(cv::Mat transformation_mat) {
//  cv::Mat orb_rotation(3, 3, CV_32F);
//  cv::Mat orb_translation(3, 1, CV_32F);
//
//  orb_rotation = transformation_mat.rowRange(0, 3).colRange(0, 3);
//  orb_translation = transformation_mat.rowRange(0, 3).col(3);
//
//  tf2::Matrix3x3 tf_camera_rotation(
//      orb_rotation.at<float>(0, 0), orb_rotation.at<float>(0, 1),
//      orb_rotation.at<float>(0, 2), orb_rotation.at<float>(1, 0),
//      orb_rotation.at<float>(1, 1), orb_rotation.at<float>(1, 2),
//      orb_rotation.at<float>(2, 0), orb_rotation.at<float>(2, 1),
//      orb_rotation.at<float>(2, 2));
//
//  tf2::Vector3 tf_camera_translation(orb_translation.at<float>(0),
//                                                  orb_translation.at<float>(1),
//                                                  orb_translation.at<float>(2));
//
//  // cout << setprecision(9) << "Rotation: " << endl << orb_rotation << endl;
//  // cout << setprecision(9) << "Translation xyz: " << orb_translation.at<float>
//  // (0) << " " << orb_translation.at<float> (1) << " " <<
//  // orb_translation.at<float> (2) << endl;
//
//  // Transform from orb coordinate system to ros coordinate system on camera
//  // coordinates
//  // tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
//  // tf_camera_translation = tf_orb_to_ros * tf_camera_translation;
//
//  // Inverse matrix
//  // tf_camera_rotation = tf_camera_rotation.transpose();
//  // tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);
//
//  // Transform from orb coordinate system to ros coordinate system on map
//  // coordinates
//  // tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
//  // tf_camera_translation = tf_orb_to_ros * tf_camera_translation;
//
//  return tf2::Transform(tf_camera_rotation, tf_camera_translation);
//}

sensor_msgs::msg::PointCloud2
tracked_mappoints_to_pointcloud(std::vector<ORB_SLAM3::MapPoint *> map_points,
                                rclcpp::Time current_frame_time) {
  const int num_channels = 3; // x y z

  if (map_points.size() == 0) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::msg::PointCloud2 cloud;

  cloud.header.stamp = current_frame_time;
  cloud.header.frame_id = map_frame_id;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = {"x", "y", "z"};

  for (int i = 0; i < num_channels; i++) {
    cloud.fields[i].name = channel_id[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count = 1;
    cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

  unsigned char *cloud_data_ptr = &(cloud.data[0]);

  for (unsigned int i = 0; i < cloud.width; i++) {
    if (map_points[i]) {

      tf2::Vector3 point_translation(map_points[i]->GetWorldPos()(0),
                                     map_points[i]->GetWorldPos()(1),
                                     map_points[i]->GetWorldPos()(2));

      point_translation = tf_orb_to_ros * point_translation;

      float data_array[num_channels] = {
          point_translation.x(), point_translation.y(), point_translation.z()};

      memcpy(cloud_data_ptr + (i * cloud.point_step), data_array,
             num_channels * sizeof(float));
    }
  }
  return cloud;
}
