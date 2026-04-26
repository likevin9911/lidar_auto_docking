#ifndef LIDAR_AUTO_DOCKING_AUTODOCK_H
#define LIDAR_AUTO_DOCKING_AUTODOCK_H

#include <ros/ros.h>
#include <angles/angles.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include "lidar_auto_docking/controller.h"
#include "lidar_auto_docking/perception.h"
#include "lidar_auto_docking/DockAction.h"
#include "lidar_auto_docking/UndockAction.h"

class DockingServer {
 public:
  explicit DockingServer(ros::NodeHandle& nh);

  void init_objects();

 private:
  typedef actionlib::SimpleActionServer<lidar_auto_docking::DockAction> DockActionServer;
  typedef actionlib::SimpleActionServer<lidar_auto_docking::UndockAction> UndockActionServer;

  ros::NodeHandle nh_;

  std::shared_ptr<DockActionServer> dock_server_;
  std::shared_ptr<BaseController> controller_;
  std::shared_ptr<DockPerception> perception_;

  void executeDock(const lidar_auto_docking::DockGoalConstPtr& goal);

  void initDockTimeout();
  bool isDockingTimedOut();
  bool continueDocking(lidar_auto_docking::DockResult& result);
  double backupDistance();
  void executeBackupSequence(ros::Rate& r);
  bool isApproachBad(double& dock_yaw);

  // Configuration
  int NUM_OF_RETRIES_;
  double DOCK_CONNECTOR_CLEARANCE_DISTANCE_;
  double DOCKED_DISTANCE_THRESHOLD_;
  double abort_distance_;
  double abort_threshold_;
  double abort_angle_;
  double correction_angle_;
  double backup_limit_;
  bool aborting_;
  int num_of_retries_;
  bool cancel_docking_;
  ros::Time deadline_docking_;
  ros::Time deadline_not_charging_;
  bool charging_timeout_set_;
  bool charging_;
};

class UndockingServer {
 public:
  explicit UndockingServer(ros::NodeHandle& nh);

  void init_objects();

 private:
  typedef actionlib::SimpleActionServer<lidar_auto_docking::UndockAction> UndockActionServer;

  ros::NodeHandle nh_;
  std::shared_ptr<UndockActionServer> undock_server_;
  std::shared_ptr<BaseController> controller_;

  void executeUndock(const lidar_auto_docking::UndockGoalConstPtr& goal);

  double DOCK_CONNECTOR_CLEARANCE_DISTANCE_;
};

#endif  // LIDAR_AUTO_DOCKING_AUTODOCK_H
