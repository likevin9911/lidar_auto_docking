#ifndef LIDAR_AUTO_DOCKING_TF2LISTENER_H
#define LIDAR_AUTO_DOCKING_TF2LISTENER_H

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>

#include <memory>
#include <string>

class tf2_listener {
 public:
  tf2_listener() : rate_(10) {
    tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
  }

  void waitTransform(std::string origin, std::string destination);

  geometry_msgs::TransformStamped getTransform(std::string origin,
                                               std::string destination);

  void transformPose(std::string tracking_frame,
                     geometry_msgs::PoseStamped& input_pose,
                     geometry_msgs::PoseStamped& output_pose);

 private:
  tf2_ros::Buffer buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  ros::Rate rate_;
};

#endif  // LIDAR_AUTO_DOCKING_TF2LISTENER_H
