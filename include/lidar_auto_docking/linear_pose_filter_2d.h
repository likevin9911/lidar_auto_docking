#ifndef LIDAR_AUTO_DOCKING_LINEAR_POSE_FILTER_H
#define LIDAR_AUTO_DOCKING_LINEAR_POSE_FILTER_H

#include <deque>
#include <memory>
#include <vector>
#include <geometry_msgs/Pose.h>

class LinearPoseFilter2D {
 public:
  LinearPoseFilter2D(const std::vector<float>& b, const std::vector<float>& a);

  void setCoeff(const std::vector<float>& b, const std::vector<float>& a);

  geometry_msgs::Pose filter(const geometry_msgs::Pose& pose);

  void reset();

  void setFilterState(const geometry_msgs::Pose& input_pose,
                      const geometry_msgs::Pose& output_pose);

  void setFilterState(const std::vector<geometry_msgs::Pose>& input_poses,
                      const std::vector<geometry_msgs::Pose>& output_poses);

 private:
  geometry_msgs::Pose originPose();

  float getUnnormalizedYaw(geometry_msgs::Pose pose, float reference_yaw);

  float getNewestOutputYaw();

  std::deque<geometry_msgs::Pose> out_;
  std::deque<geometry_msgs::Pose> in_;
  std::deque<float> yaw_out_;
  std::deque<float> yaw_in_;
  std::vector<float> b_;
  std::vector<float> a_;
};

typedef std::shared_ptr<LinearPoseFilter2D> LinearPoseFilter2DPtr;

#endif  // LIDAR_AUTO_DOCKING_LINEAR_POSE_FILTER_H
