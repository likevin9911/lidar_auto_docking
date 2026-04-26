#ifndef LIDAR_AUTO_DOCKING_PERCEPTION_H
#define LIDAR_AUTO_DOCKING_PERCEPTION_H

#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>

#include <mutex>
#include <string>
#include <vector>

#include "lidar_auto_docking/dock_candidate.h"
#include "lidar_auto_docking/icp_2d.h"
#include "lidar_auto_docking/laser_processor.h"
#include "lidar_auto_docking/linear_pose_filter_2d.h"
#include "lidar_auto_docking/tf2listener.h"

class DockPerception {
 public:
  explicit DockPerception(ros::NodeHandle& nh);

  bool start(const geometry_msgs::PoseStamped& pose);
  bool stop();
  bool getPose(geometry_msgs::PoseStamped& pose, std::string frame = "");

 private:
  void callback(const sensor_msgs::LaserScan::ConstPtr& scan);

  DockCandidatePtr extract(laser_processor::SampleSet* cluster);

  double fit(const DockCandidatePtr& candidate, geometry_msgs::Pose& pose);

  bool isValid(const tf2::Quaternion& q);

  ros::Subscriber scan_sub_;
  tf2_listener listener_;

  bool running_;
  bool debug_;
  bool found_dock_;

  std::string tracking_frame_;
  geometry_msgs::PoseStamped dock_;
  ros::Time dock_stamp_;
  std::mutex dock_mutex_;

  double max_alignment_error_;

  std::vector<geometry_msgs::Point> ideal_cloud_;
  std::vector<geometry_msgs::Point> front_cloud_;

  LinearPoseFilter2DPtr dock_pose_filter_;
};

#endif  // LIDAR_AUTO_DOCKING_PERCEPTION_H
