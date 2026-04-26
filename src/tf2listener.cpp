#include <lidar_auto_docking/tf2listener.h>

void tf2_listener::waitTransform(std::string origin, std::string destination) {
  std::string warning_msg;
  while (ros::ok() &&
         !buffer_.canTransform(origin, destination, ros::Time(0),
                               &warning_msg)) {
    ROS_WARN_THROTTLE(1.0, "Waiting for transform %s -> %s: %s",
                      origin.c_str(), destination.c_str(), warning_msg.c_str());
    rate_.sleep();
  }
}

geometry_msgs::TransformStamped tf2_listener::getTransform(
    std::string origin, std::string destination) {
  return buffer_.lookupTransform(origin, destination, ros::Time(0));
}

void tf2_listener::transformPose(std::string tracking_frame,
                                 geometry_msgs::PoseStamped& input_pose,
                                 geometry_msgs::PoseStamped& output_pose) {
  geometry_msgs::TransformStamped corrective_transform =
      getTransform(tracking_frame, input_pose.header.frame_id);
  tf2::doTransform(input_pose, output_pose, corrective_transform);
}
