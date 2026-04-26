#ifndef LIDAR_AUTO_DOCKING_CONTROLLER_H
#define LIDAR_AUTO_DOCKING_CONTROLLER_H

#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "lidar_auto_docking/tf2listener.h"

class BaseController {
 public:
  explicit BaseController(ros::NodeHandle& nh);

  /**
   * @brief Implements something loosely based on "A Smooth Control Law for
   * Graceful Motion of Differential Wheeled Mobile Robots in 2D Environments"
   * by Park and Kuipers, ICRA 2011
   * @returns true if base has reached goal.
   */
  bool approach(const geometry_msgs::PoseStamped& target);

  /**
   * @brief Back off dock, then rotate.
   * @param distance        Distance in meters to backup.
   * @param rotate_distance Amount of angle in radians for the robot to yaw.
   */
  bool backup(double distance, double rotate_distance);

  /**
   * @brief Get the last command sent
   */
  bool getCommand(geometry_msgs::Twist& command);

  /** @brief Send stop command to robot base */
  void stop();

 private:
  ros::Publisher cmd_vel_pub_;
  ros::Publisher path_pub_;

  tf2_listener listener_;
  geometry_msgs::Twist command_;

  // Approach controller parameters
  double k1_;
  double k2_;
  double min_velocity_;
  double max_velocity_;
  double max_angular_velocity_;
  double beta_;
  double lambda_;
  double dist_;

  // Backup controller parameters
  geometry_msgs::PoseStamped start_;
  bool ready_;
  bool turning_;
};

#endif  // LIDAR_AUTO_DOCKING_CONTROLLER_H
