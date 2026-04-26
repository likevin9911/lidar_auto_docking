#include <lidar_auto_docking/autodock.h>

// ==================== DockingServer ====================

DockingServer::DockingServer(ros::NodeHandle& nh) : nh_(nh) {
  ros::NodeHandle pnh("~");
  pnh.param("retries", NUM_OF_RETRIES_, 5);
  pnh.param("abort_distance", abort_distance_, 0.32);
  pnh.param("abort_angle", abort_angle_, 0.090);
  pnh.param("y_abort_threshold", abort_threshold_, 0.04);
  pnh.param("connector_clearance_distance", DOCK_CONNECTOR_CLEARANCE_DISTANCE_, 0.4);
  pnh.param("docked_distance_threshold", DOCKED_DISTANCE_THRESHOLD_, 0.30);

  dock_server_ = std::make_shared<DockActionServer>(
      nh_, "Dock",
      boost::bind(&DockingServer::executeDock, this, _1),
      false);
  dock_server_->start();
}

void DockingServer::init_objects() {
  perception_ = std::make_shared<DockPerception>(nh_);
  controller_ = std::make_shared<BaseController>(nh_);
}

void DockingServer::initDockTimeout() {
  deadline_docking_ = ros::Time::now() + ros::Duration(120.0);
  num_of_retries_ = NUM_OF_RETRIES_;
}

bool DockingServer::isDockingTimedOut() {
  return (ros::Time::now() > deadline_docking_ || !num_of_retries_);
}

bool DockingServer::continueDocking(lidar_auto_docking::DockResult& result) {
  if (charging_) {
    result.docked = true;
    dock_server_->setSucceeded(result);
    ROS_INFO("DOCK REACHED!");
    return false;
  } else if (isDockingTimedOut() || cancel_docking_) {
    result.docked = false;
    dock_server_->setAborted(result);
    ROS_INFO("Docking Cancelled/Timed out");
    return false;
  } else if (dock_server_->isPreemptRequested()) {
    dock_server_->setPreempted();
    return false;
  }
  return true;
}

double DockingServer::backupDistance() {
  double distance = 1.0;
  distance *= 1.5 * fabs(correction_angle_);
  double retry_constant =
      2 - static_cast<float>(num_of_retries_) / NUM_OF_RETRIES_;
  retry_constant = std::max(1.0, std::min(2.0, retry_constant));
  distance *= retry_constant;
  backup_limit_ = std::min(1.0, backup_limit_);
  distance = std::max(0.2, std::min(backup_limit_, distance));
  return distance;
}

void DockingServer::executeBackupSequence(ros::Rate& r) {
  ROS_ERROR("Poor Approach! Backing up!");
  while (!controller_->backup(DOCK_CONNECTOR_CLEARANCE_DISTANCE_,
                              correction_angle_)) {
    if (isDockingTimedOut()) return;
    r.sleep();
  }
  while (!controller_->backup(backupDistance(), 0.0)) {
    if (isDockingTimedOut()) return;
    r.sleep();
  }
}

bool DockingServer::isApproachBad(double& dock_yaw) {
  geometry_msgs::PoseStamped dock_pose_base_link;
  perception_->getPose(dock_pose_base_link, "base_link");

  dock_yaw = angles::normalize_angle(
      tf2::getYaw(dock_pose_base_link.pose.orientation));

  if (dock_pose_base_link.pose.position.x < abort_distance_ &&
      dock_pose_base_link.pose.position.x > DOCKED_DISTANCE_THRESHOLD_) {
    if (fabs(dock_pose_base_link.pose.position.y) > abort_threshold_ ||
        fabs(dock_yaw) > abort_angle_) {
      return true;
    }
  }
  return false;
}

void DockingServer::executeDock(
    const lidar_auto_docking::DockGoalConstPtr& goal) {
  ROS_INFO("Executing dock goal");

  ros::Rate loop_rate(50);
  lidar_auto_docking::DockResult result;
  lidar_auto_docking::DockFeedback feedback;

  result.docked = false;
  aborting_ = false;
  charging_timeout_set_ = false;
  cancel_docking_ = false;
  charging_ = false;

  // Start perception with goal dock pose
  perception_->start(goal->dock_pose);
  initDockTimeout();

  geometry_msgs::PoseStamped dock_pose_base_link;
  ROS_INFO("Finding Dock...");

  while (!perception_->getPose(dock_pose_base_link, "base_link")) {
    if (!continueDocking(result)) {
      ROS_ERROR("Docking failed: Initial dock not found.");
      return;
    }
    loop_rate.sleep();
  }

  ROS_INFO("Pre-orienting robot toward dock");
  double dock_yaw = angles::normalize_angle(
      tf2::getYaw(dock_pose_base_link.pose.orientation));

  if (!std::isfinite(dock_yaw)) {
    ROS_ERROR("Dock yaw is invalid: %f", dock_yaw);
    cancel_docking_ = true;
  } else if (ros::ok() && continueDocking(result)) {
    backup_limit_ =
        std::sqrt(std::pow(dock_pose_base_link.pose.position.x, 2) +
                  std::pow(dock_pose_base_link.pose.position.y, 2));
    backup_limit_ *= 0.9;

    while (!controller_->backup(0.0, dock_yaw) &&
           continueDocking(result) && ros::ok()) {
      loop_rate.sleep();
    }
  }

  controller_->stop();

  // Main docking loop
  while (ros::ok() && continueDocking(result)) {
    if (perception_->getPose(feedback.dock_pose)) {
      if (aborting_) {
        executeBackupSequence(loop_rate);
        aborting_ = false;
        num_of_retries_--;
      } else {
        if (isApproachBad(correction_angle_)) {
          controller_->stop();
          aborting_ = true;
        } else {
          geometry_msgs::PoseStamped dock_x_distance;
          perception_->getPose(dock_x_distance, "base_link");
          ROS_INFO_THROTTLE(1.0, "Distance to dock: %.3f",
                            dock_x_distance.pose.position.x);

          if (dock_x_distance.pose.position.x <= DOCKED_DISTANCE_THRESHOLD_)
            charging_ = true;
          else
            charging_ = false;

          controller_->approach(feedback.dock_pose);
        }
      }
      controller_->getCommand(feedback.command);
      dock_server_->publishFeedback(feedback);
    }
    loop_rate.sleep();
  }

  controller_->stop();
  perception_->stop();
}

// ==================== UndockingServer ====================

UndockingServer::UndockingServer(ros::NodeHandle& nh) : nh_(nh) {
  ros::NodeHandle pnh("~");
  pnh.param("connector_clearance_distance",
            DOCK_CONNECTOR_CLEARANCE_DISTANCE_, 0.4);

  undock_server_ = std::make_shared<UndockActionServer>(
      nh_, "Undock",
      boost::bind(&UndockingServer::executeUndock, this, _1),
      false);
  undock_server_->start();
}

void UndockingServer::init_objects() {
  controller_ = std::make_shared<BaseController>(nh_);
}

void UndockingServer::executeUndock(
    const lidar_auto_docking::UndockGoalConstPtr& goal) {
  ROS_INFO("Executing undock goal");

  ros::Rate loop_rate(50);
  lidar_auto_docking::UndockResult result;
  lidar_auto_docking::UndockFeedback feedback;
  result.undocked = false;

  double backup = DOCK_CONNECTOR_CLEARANCE_DISTANCE_;
  double turn = goal->rotate_in_place ? 3.1 : 0.0;

  ros::Time timeout = ros::Time::now() + ros::Duration(50.0);
  controller_->stop();

  while (ros::ok()) {
    if (undock_server_->isPreemptRequested()) {
      undock_server_->setPreempted();
      return;
    }

    if (controller_->backup(backup, turn)) {
      ROS_INFO("Undock Successful!");
      result.undocked = true;
      controller_->stop();
      undock_server_->setSucceeded(result);
      return;
    }

    if (ros::Time::now() > timeout) {
      controller_->stop();
      ROS_WARN("Undock timed out!");
      undock_server_->setAborted(result);
      return;
    }

    controller_->getCommand(feedback.command);
    undock_server_->publishFeedback(feedback);
    loop_rate.sleep();
  }
}

// ==================== main ====================

int main(int argc, char** argv) {
  ros::init(argc, argv, "auto_dock");
  ros::NodeHandle nh;

  DockingServer docking_server(nh);
  docking_server.init_objects();

  UndockingServer undocking_server(nh);
  undocking_server.init_objects();

  // Multi-threaded spinner to handle both action servers
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
