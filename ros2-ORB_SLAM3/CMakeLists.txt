cmake_minimum_required(VERSION 3.5)
project(ros2_orbslam3)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(image_transport REQUIRED)
find_package(Pangolin REQUIRED)

set(ORB_SLAM3_DIR $ENV{ORB_SLAM3_DIR})

include_directories(
    ${ORB_SLAM3_DIR}
    ${ORB_SLAM3_DIR}/include
    ${ORB_SLAM3_DIR}/include/CameraModels
    ${ORB_SLAM3_DIR}/Thirdparty/Sophus)

set(LIBS 
    ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${image_transport_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${tf2_LIBRARIES}
    -lboost_serialization
    -lcrypto)

add_executable(slam src/slam.cpp src/common.cpp)
target_link_libraries(slam ${LIBS})
ament_target_dependencies(slam rclcpp sensor_msgs cv_bridge message_filters tf2_ros tf2 image_transport Pangolin)
install(TARGETS slam
  DESTINATION lib/${PROJECT_NAME})

#add_executable(mono src/mono.cpp src/common.cpp)
#target_link_libraries(mono ${LIBS})
#ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge message_filters tf2_ros tf2 image_transport Pangolin)
#install(TARGETS mono
#  DESTINATION lib/${PROJECT_NAME})

add_executable(rgbd src/rgbd.cpp src/common.cpp)
target_link_libraries(rgbd ${LIBS})
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters tf2_ros tf2 image_transport Pangolin)
install(TARGETS rgbd
  DESTINATION lib/${PROJECT_NAME})

ament_package()
