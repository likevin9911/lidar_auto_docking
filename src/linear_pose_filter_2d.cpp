#include <lidar_auto_docking/linear_pose_filter_2d.h>

#include <algorithm>
#include <angles/angles.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <iostream>

LinearPoseFilter2D::LinearPoseFilter2D(const std::vector<float>& b,
                                       const std::vector<float>& a) {
  setCoeff(b, a);
}

void LinearPoseFilter2D::setCoeff(const std::vector<float>& b,
                                  const std::vector<float>& a) {
  size_t N = std::min(b.size(), a.size());
  b_.resize(N);
  a_.resize(N);
  std::copy_backward(b.end() - N, b.end(), b_.end());
  std::copy_backward(a.end() - N, a.end(), a_.end());

  int K = in_.size() - b_.size();
  if (K > 0) {
    for (int i = 0; i < K; i++) {
      in_.pop_front(); out_.pop_front();
      yaw_in_.pop_front(); yaw_out_.pop_front();
    }
  } else {
    for (int i = 0; i > K; i--) {
      in_.push_front(originPose()); out_.push_front(originPose());
      yaw_in_.push_front(tf2::getYaw(originPose().orientation));
      yaw_out_.push_front(tf2::getYaw(originPose().orientation));
    }
  }
}

geometry_msgs::Pose LinearPoseFilter2D::filter(
    const geometry_msgs::Pose& pose) {
  in_.push_back(pose);
  yaw_in_.push_back(getUnnormalizedYaw(pose, getNewestOutputYaw()));
  out_.push_back(originPose());
  yaw_out_.push_back(0.0f);
  in_.pop_front(); out_.pop_front();
  yaw_in_.pop_front(); yaw_out_.pop_front();

  std::deque<geometry_msgs::Pose>::iterator outn = --out_.end();
  std::deque<float>::iterator yaw_outn = --yaw_out_.end();

  size_t N = b_.size() - 1;
  for (size_t i = 0; i < (N + 1); i++) {
    outn->position.x += b_[i] * in_[N - i].position.x;
    outn->position.y += b_[i] * in_[N - i].position.y;
    *yaw_outn += b_[i] * yaw_in_[N - i];
    if (i > 0) {
      outn->position.x -= a_[i] * out_[N - i].position.x;
      outn->position.y -= a_[i] * out_[N - i].position.y;
      *yaw_outn -= a_[i] * yaw_out_[N - i];
    }
  }

  outn->orientation.z = sin(*yaw_outn / 2.0);
  outn->orientation.w = cos(*yaw_outn / 2.0);
  return *outn;
}

void LinearPoseFilter2D::reset() {
  setFilterState(originPose(), originPose());
}

void LinearPoseFilter2D::setFilterState(
    const geometry_msgs::Pose& input_pose,
    const geometry_msgs::Pose& output_pose) {
  float yaw_out = tf2::getYaw(output_pose.orientation);
  std::fill(out_.begin(), out_.end(), output_pose);
  std::fill(yaw_out_.begin(), yaw_out_.end(), yaw_out);
  std::fill(in_.begin(), in_.end(), input_pose);
  std::fill(yaw_in_.begin(), yaw_in_.end(),
            getUnnormalizedYaw(input_pose, yaw_out));
}

void LinearPoseFilter2D::setFilterState(
    const std::vector<geometry_msgs::Pose>& input_poses,
    const std::vector<geometry_msgs::Pose>& output_poses) {
  std::vector<geometry_msgs::Pose>::const_iterator earliest_pose_out;
  if (output_poses.size() <= out_.size())
    earliest_pose_out = output_poses.begin();
  else
    earliest_pose_out = output_poses.end() - out_.size();

  std::copy_backward(earliest_pose_out, output_poses.end(), out_.end());

  std::deque<float>::iterator yaw_out = yaw_out_.end();
  std::deque<float>::iterator yaw_previous = yaw_out_.end();
  std::vector<geometry_msgs::Pose>::const_iterator pose_out = output_poses.end();
  while (pose_out != earliest_pose_out) {
    if (yaw_previous == yaw_out_.end())
      *(--yaw_previous) = tf2::getYaw((--output_poses.end())->orientation);
    *(--yaw_out) = getUnnormalizedYaw(*(--pose_out), *(yaw_previous--));
  }

  std::vector<geometry_msgs::Pose>::const_iterator earliest_pose_in;
  if (input_poses.size() <= in_.size())
    earliest_pose_in = input_poses.begin();
  else
    earliest_pose_in = input_poses.end() - in_.size();

  std::copy_backward(earliest_pose_in, input_poses.end(), in_.end());

  yaw_previous = yaw_out_.end();
  std::deque<float>::iterator yaw_in = yaw_in_.end();
  std::vector<geometry_msgs::Pose>::const_iterator pose_in = input_poses.end();
  while (pose_in != earliest_pose_in)
    *(--yaw_in) = getUnnormalizedYaw(*(--pose_in), *(--yaw_previous));
}

geometry_msgs::Pose LinearPoseFilter2D::originPose() {
  geometry_msgs::Pose origin;
  origin.position.x = 0; origin.position.y = 0; origin.position.z = 0;
  origin.orientation.x = 0; origin.orientation.y = 0;
  origin.orientation.z = 0; origin.orientation.w = 1;
  return origin;
}

float LinearPoseFilter2D::getUnnormalizedYaw(geometry_msgs::Pose pose,
                                             float reference_yaw) {
  float yaw = tf2::getYaw(pose.orientation);
  return reference_yaw + angles::normalize_angle(yaw - reference_yaw);
}

float LinearPoseFilter2D::getNewestOutputYaw() {
  if (yaw_out_.empty()) return 0.0f;
  return *(--yaw_out_.end());
}
