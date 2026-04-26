#include <angles/angles.h>
#include <lidar_auto_docking/controller.h>

#include <algorithm>
#include <cmath>
#include <list>
#include <vector>

BaseController::BaseController(ros::NodeHandle& nh)
    : ready_(false), turning_(false) {
  path_pub_ = nh.advertise<nav_msgs::Path>("path", 10);
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  k1_ = 3;
  k2_ = 2;
  min_velocity_ = 0.06;
  max_velocity_ = 0.06;
  max_angular_velocity_ = 1.0;
  beta_ = 0.2;
  lambda_ = 2.0;
  dist_ = 0.4;
}

bool BaseController::approach(const geometry_msgs::PoseStamped& target) {
  geometry_msgs::PoseStamped pose = target;
  {
    double theta = angles::normalize_angle(tf2::getYaw(pose.pose.orientation));
    if (!std::isfinite(theta)) {
      ROS_ERROR("Invalid approach target for docking");
      stop();
      return true;
    }
    pose.pose.position.x += cos(theta) * -dist_;
    pose.pose.position.y += sin(theta) * -dist_;
  }

  try {
    pose.header.stamp = ros::Time(0);
    listener_.transformPose("base_link", pose, pose);
  } catch (const tf2::TransformException& ex) {
    ROS_ERROR("Couldn't get transform from dock to base_link: %s", ex.what());
    stop();
    return false;
  }

  double r = std::sqrt(pose.pose.position.x * pose.pose.position.x +
                       pose.pose.position.y * pose.pose.position.y);

  if (r < 0.3) dist_ = 0.0;
  if (r < 0.01) return true;

  double delta = std::atan2(-pose.pose.position.y, pose.pose.position.x);
  double theta = angles::normalize_angle(
      tf2::getYaw(pose.pose.orientation) + delta);

  double a = atan(-k1_ * theta);
  double k = -1.0 / r *
             (k2_ * (delta - a) +
              (1 + (k1_ / 1 + ((k1_ * theta) * (k1_ * theta)))) * sin(delta));

  double v = max_velocity_ / (1 + beta_ * std::pow(fabs(k), lambda_));

  if (r < 0.75)
    v = std::max(min_velocity_, std::min(std::min(r, max_velocity_), v));
  else
    v = std::min(max_velocity_, std::max(min_velocity_, v));

  double w = k * v;
  double bounded_w = std::min(max_angular_velocity_,
                               std::max(-max_angular_velocity_, w));
  if (w != 0.0) v *= (bounded_w / w);

  command_.linear.x = v;
  command_.angular.z = bounded_w;
  cmd_vel_pub_.publish(command_);

  // Publish path for debugging
  nav_msgs::Path plan;
  plan.header.stamp = ros::Time::now();
  plan.header.frame_id = "base_link";
  geometry_msgs::PoseStamped path_pose;
  path_pose.header.frame_id = "base_link";
  path_pose.pose.orientation.w = 1.0;
  plan.poses.push_back(path_pose);
  double yaw = 0.0;
  for (int i = 0; i < 20; i++) {
    path_pose.pose.position.x += 0.1 * command_.linear.x * cos(yaw);
    path_pose.pose.position.y += 0.1 * command_.linear.x * sin(yaw);
    yaw += 0.1 * command_.angular.z;
    path_pose.pose.orientation.z = sin(theta / 2.0);
    path_pose.pose.orientation.w = cos(theta / 2.0);
    double dx = path_pose.pose.position.x - pose.pose.position.x;
    double dy = path_pose.pose.position.y - pose.pose.position.y;
    if ((dx * dx + dy * dy) < 0.005) break;
    plan.poses.push_back(path_pose);
  }
  plan.poses.push_back(pose);
  path_pub_.publish(plan);

  return false;
}

bool BaseController::backup(double distance, double rotate_distance) {
  if (!std::isfinite(distance) || !std::isfinite(rotate_distance)) {
    ROS_ERROR("Backup parameters are not valid.");
    stop();
    return true;
  }

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.orientation.w = 1.0;

  try {
    listener_.waitTransform("map", pose.header.frame_id);
    listener_.transformPose("map", pose, pose);
  } catch (const tf2::TransformException& ex) {
    ROS_ERROR("Couldn't get transform from base_link to map: %s", ex.what());
    stop();
    return false;
  }

  if (!ready_) {
    start_ = pose;
    turning_ = false;
    ready_ = true;
  }

  if (turning_) {
    double theta = angles::normalize_angle(
        tf2::getYaw(pose.pose.orientation) -
        tf2::getYaw(start_.pose.orientation));
    double error = angles::normalize_angle(rotate_distance - theta);

    if (fabs(error) < 0.05) {
      stop();
      return true;
    } else if (rotate_distance > 0.0)
      command_.angular.z = std::min(0.6, fabs(error) * 1.3 + 0.1);
    else
      command_.angular.z = std::max(-0.6, -(fabs(error) * 1.3 + 0.1));
  } else {
    double dx = pose.pose.position.x - start_.pose.position.x;
    double dy = pose.pose.position.y - start_.pose.position.y;
    if ((dx * dx + dy * dy) > (distance * distance)) {
      if (rotate_distance == 0.0) {
        stop();
        return true;
      } else {
        turning_ = true;
        command_.linear.x = 0.0;
      }
    } else {
      command_.linear.x = -0.1;
    }
  }

  cmd_vel_pub_.publish(command_);
  return false;
}

bool BaseController::getCommand(geometry_msgs::Twist& command) {
  command = command_;
  return true;
}

void BaseController::stop() {
  command_ = geometry_msgs::Twist();
  cmd_vel_pub_.publish(command_);
  ready_ = false;
  dist_ = 0.4;
}
