/*
 * Copyright 2015 Fetch Robotics Inc.
 * Author: Michael Ferguson, Sriramvarun Nidamarthy
 * Ported to ROS1.
 */

#include <angles/angles.h>
#include <lidar_auto_docking/icp_2d.h>
#include <lidar_auto_docking/perception.h>

#include <iostream>
#include <list>
#include <queue>
#include <string>
#include <vector>

double getPoseDistance(const geometry_msgs::Pose a,
                       const geometry_msgs::Pose b) {
  double dx = a.position.x - b.position.x;
  double dy = a.position.y - b.position.y;
  return sqrt(dx * dx + dy * dy);
}

DockPerception::DockPerception(ros::NodeHandle& nh)
    : running_(false),
      tracking_frame_("base_link"),
      found_dock_(false) {
  debug_ = false;

  // Butterworth filter coefficients
  // [b, a] = butter(2, 10/25)
  float b_arr[] = {0.20657, 0.41314, 0.20657};
  float a_arr[] = {1.00000, -0.36953, 0.19582};
  std::vector<float> b(b_arr, b_arr + sizeof(b_arr) / sizeof(float));
  std::vector<float> a(a_arr, a_arr + sizeof(a_arr) / sizeof(float));
  dock_pose_filter_.reset(new LinearPoseFilter2D(b, a));

  max_alignment_error_ = 0.01;

  // Create ideal dock cloud
  // Front face is 300mm long
  for (double y = -0.15; y <= 0.15; y += 0.001) {
    geometry_msgs::Point p;
    p.x = p.z = 0.0;
    p.y = y;
    ideal_cloud_.push_back(p);
    front_cloud_.push_back(p);
  }
  // Each side is 100mm long at 45 degree angle
  for (double x = 0.0; x < 0.05; x += 0.001) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = 0.15 + x;
    p.z = 0.0;
    ideal_cloud_.push_back(p);
    p.y = -0.15 - x;
    ideal_cloud_.insert(ideal_cloud_.begin(), p);
  }

  scan_sub_ = nh.subscribe("lidar/scan", 10,
                            &DockPerception::callback, this);
  ROS_INFO("Dock perception initialized");
}

bool DockPerception::start(const geometry_msgs::PoseStamped& pose) {
  running_ = false;
  found_dock_ = false;
  dock_ = pose;
  running_ = true;
  return true;
}

bool DockPerception::stop() {
  running_ = false;
  return true;
}

bool DockPerception::getPose(geometry_msgs::PoseStamped& pose,
                             std::string frame) {
  std::lock_guard<std::mutex> lock(dock_mutex_);

  if (!found_dock_) return false;

  tf2::Quaternion q;
  tf2::fromMsg(dock_.pose.orientation, q);
  if (!isValid(q)) {
    ROS_WARN("Dock orientation invalid.");
    return false;
  }

  pose = dock_;

  if (frame != "") {
    try {
      listener_.waitTransform(frame, pose.header.frame_id);
      listener_.transformPose(frame, pose, pose);
    } catch (const tf2::TransformException& ex) {
      ROS_WARN("Couldn't transform dock pose: %s", ex.what());
      return false;
    }
  }

  return found_dock_;
}

void DockPerception::callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  if (!running_) return;

  std::lock_guard<std::mutex> lock(dock_mutex_);

  // If goal is invalid, set to a point directly ahead of robot
  if (dock_.header.frame_id == "" ||
      (dock_.pose.orientation.z == 0.0 && dock_.pose.orientation.w == 0.0)) {
    dock_.header = scan->header;
    for (size_t i = scan->ranges.size() / 2; i < scan->ranges.size(); i++) {
      if (std::isfinite(scan->ranges[i])) {
        double angle = scan->angle_min + i * scan->angle_increment;
        dock_.pose.position.x = cos(angle) * scan->ranges[i];
        dock_.pose.position.y = sin(angle) * scan->ranges[i];
        dock_.pose.orientation.x = 1.0;
        dock_.pose.orientation.y = 0.0;
        dock_.pose.orientation.z = 0.0;
        dock_.pose.orientation.w = 0.0;
        break;
      }
    }
  }

  // Transform goal into tracking frame
  if (dock_.header.frame_id != tracking_frame_) {
    try {
      listener_.waitTransform(tracking_frame_, dock_.header.frame_id);
      listener_.transformPose(tracking_frame_, dock_, dock_);
    } catch (const tf2::TransformException& ex) {
      ROS_WARN("Couldn't transform dock pose to tracking frame: %s", ex.what());
      return;
    }
  }

  // Cluster the laser scan
  laser_processor::ScanMask mask;
  laser_processor::ScanProcessor processor(*scan, mask);
  processor.splitConnected(0.04);
  processor.removeLessThan(5);

  // Sort clusters by distance to last dock pose
  std::priority_queue<DockCandidatePtr, std::vector<DockCandidatePtr>,
                      CompareCandidates>
      candidates;
  for (std::list<laser_processor::SampleSet*>::iterator i =
           processor.getClusters().begin();
       i != processor.getClusters().end(); i++) {
    DockCandidatePtr c = extract(*i);
    if (c && c->valid(found_dock_)) {
      candidates.push(c);
    }
  }

  // Find best fitting candidate via ICP
  DockCandidatePtr best;
  geometry_msgs::Pose best_pose;
  while (!candidates.empty()) {
    geometry_msgs::Pose pose = dock_.pose;
    double score = fit(candidates.top(), pose);
    if (score >= 0) {
      best = candidates.top();
      best_pose = pose;
      break;
    }
    candidates.pop();
  }

  if (best.use_count() == 0) {
    ROS_DEBUG_THROTTLE(1.0, "Dock not found in scan");
    return;
  }

  // Update stamp
  dock_.header.stamp = scan->header.stamp;
  dock_.header.frame_id = tracking_frame_;

  if (!found_dock_) {
    dock_.pose = best_pose;
    dock_pose_filter_->reset();
    dock_pose_filter_->setFilterState(dock_.pose, dock_.pose);
  } else {
    double d = getPoseDistance(dock_.pose, best_pose);
    if (d > 0.05) {
      ROS_WARN_THROTTLE(1.0, "Dock pose jumped: %.3f", d);
      return;
    }
  }

  dock_.pose = dock_pose_filter_->filter(best_pose);
  dock_stamp_ = scan->header.stamp;
  found_dock_ = true;
}

DockCandidatePtr DockPerception::extract(laser_processor::SampleSet* cluster) {
  DockCandidatePtr candidate = std::make_shared<DockCandidate>();

  geometry_msgs::TransformStamped t_frame;
  tf2::Stamped<tf2::Transform> tf2_t_frame;

  try {
    listener_.waitTransform(tracking_frame_, cluster->header.frame_id);
    t_frame = listener_.getTransform(tracking_frame_,
                                     cluster->header.frame_id);
  } catch (const tf2::TransformException& ex) {
    ROS_ERROR("Could not transform cluster point: %s", ex.what());
    return candidate;
  }

  tf2::fromMsg(t_frame, tf2_t_frame);

  for (laser_processor::SampleSet::iterator p = cluster->begin();
       p != cluster->end(); p++) {
    geometry_msgs::PointStamped pt;
    pt.header = cluster->header;

    tf2::Vector3 tf2_point((*p)->x, (*p)->y, 0);
    tf2_point = tf2_t_frame * tf2_point;
    pt.point.x = tf2_point.getX();
    pt.point.y = tf2_point.getY();
    pt.point.z = 0;
    candidate->points.push_back(pt.point);
  }

  geometry_msgs::Point centroid = icp_2d::getCentroid(candidate->points);
  double dx = centroid.x - dock_.pose.position.x;
  double dy = centroid.y - dock_.pose.position.y;
  candidate->dist = (dx * dx + dy * dy);

  return candidate;
}

double DockPerception::fit(const DockCandidatePtr& candidate,
                           geometry_msgs::Pose& pose) {
  geometry_msgs::Transform transform;
  transform.translation.x = pose.position.x;
  transform.translation.y = pose.position.y;
  transform.rotation = pose.orientation;

  tf2::Quaternion yaw_converter;
  tf2::Quaternion init_pose, cand_pose;
  tf2::fromMsg(pose.orientation, init_pose);

  if (!isValid(init_pose)) {
    ROS_ERROR("Initial dock orientation estimate is invalid.");
    return -1.0;
  }

  double fitness = icp_2d::alignICP(ideal_cloud_, candidate->points, transform);
  tf2::fromMsg(transform.rotation, cand_pose);

  if (!isValid(cand_pose)) {
    ROS_WARN("Dock candidate orientation estimate is invalid.");
  }

  // Flip if orientation seems reversed
  if (fabs(angles::normalize_angle(tf2::getYaw(
          cand_pose.inverse() * init_pose))) > 3.1415 * (2.0 / 3.0)) {
    double yaw_angle = 3.1415 + tf2::getYaw(transform.rotation);
    yaw_converter.setRPY(0, 0, yaw_angle);
    transform.rotation.x = yaw_converter.x();
    transform.rotation.y = yaw_converter.y();
    transform.rotation.z = yaw_converter.z();
    transform.rotation.w = yaw_converter.w();
  }

  if (fitness >= 0.0) {
    double retry = 5;
    tf2::fromMsg(transform.rotation, cand_pose);
    while (retry-- && (fitness > max_alignment_error_ ||
                       fabs(angles::normalize_angle(tf2::getYaw(
                           cand_pose.inverse() * init_pose))) > 3.1415 / 4.0)) {
      transform.translation.x +=
          retry * (0.75 / 100.0) * static_cast<double>((rand() % 200) - 100);
      transform.translation.y +=
          retry * (0.75 / 100.0) * static_cast<double>((rand() % 200) - 100);
      yaw_converter.setRPY(
          0, 0,
          retry * (0.28 / 100.0) * double((rand() % 200) - 100) +
              tf2::getYaw(transform.rotation));
      transform.rotation.x = yaw_converter.x();
      transform.rotation.y = yaw_converter.y();
      transform.rotation.z = yaw_converter.z();
      transform.rotation.w = yaw_converter.w();

      fitness = icp_2d::alignICP(ideal_cloud_, candidate->points, transform);

      tf2::fromMsg(transform.rotation, cand_pose);
      if (fabs(angles::normalize_angle(tf2::getYaw(
              cand_pose.inverse() * init_pose))) > 3.1415 * (2.0 / 3.0)) {
        yaw_converter.setRPY(0, 0,
                             (3.1415 + tf2::getYaw(transform.rotation)));
        transform.rotation.x = yaw_converter.x();
        transform.rotation.y = yaw_converter.y();
        transform.rotation.z = yaw_converter.z();
        transform.rotation.w = yaw_converter.w();
      }
    }

    tf2::fromMsg(transform.rotation, cand_pose);
    if (!isValid(cand_pose)) {
      ROS_ERROR("Dock candidate orientation estimate is invalid after retries.");
      return -1.0;
    }
    if (fabs(angles::normalize_angle(
            tf2::getYaw(cand_pose.inverse() * init_pose))) > 3.1415 / 2.0) {
      fitness = -1.0;
    }

    if (!found_dock_ && fabs(fitness) > max_alignment_error_) {
      fitness = -1.0;
    }

    if (candidate->width() < 0.375) {
      ROS_WARN_THROTTLE(1.0, "Dock candidate width unreliable, using heading from pose.");
      transform.rotation = pose.orientation;
      fitness = 0.001234;
    }

    candidate->points = icp_2d::transform(
        ideal_cloud_, transform.translation.x, transform.translation.y,
        icp_2d::thetaFromQuaternion(transform.rotation));

    pose.position.x = transform.translation.x;
    pose.position.y = transform.translation.y;
    pose.position.z = transform.translation.z;
    pose.orientation = transform.rotation;

    return fitness;
  }

  ROS_WARN("ICP did not converge");
  return -1.0;
}

bool DockPerception::isValid(const tf2::Quaternion& q) {
  return 1e-3 >= fabs(1.0 - q.length());
}
