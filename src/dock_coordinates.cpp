#include <lidar_auto_docking/perception.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Joy.h>

#include "lidar_auto_docking/Initdock.h"
#include "lidar_auto_docking/tf2listener.h"

class DockCoordinates {
 public:
  explicit DockCoordinates(ros::NodeHandle& nh) : nh_(nh), found_dock_(false) {
    tbr_ = std::make_shared<tf2_ros::TransformBroadcaster>();

    publisher_ = nh_.advertise<lidar_auto_docking::Initdock>("init_dock", 10);

    int reset_goal_button;
    ros::NodeHandle pnh("~");
    pnh.param("reset_goal_button", reset_goal_button, 3);
    reset_goal_button_ = reset_goal_button;

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(
        "joy", 10,
        [this](const sensor_msgs::Joy::ConstPtr& msg) {
          if ((int)msg->buttons.size() > reset_goal_button_ &&
              msg->buttons[reset_goal_button_]) {
            ROS_INFO("Resetting initial dock estimate");
            found_dock_ = false;
            perception_->stop();
            update_init_dock(init_dock_pose_);
            perception_->start(init_dock_pose_);
          }
        });
  }

  void init_objects() {
    perception_ = std::make_shared<DockPerception>(nh_);
  }

  void update_init_dock(geometry_msgs::PoseStamped& idp) {
    tf2_listen_.waitTransform("map", "base_link");
    geometry_msgs::PoseStamped fake_dock;
    fake_dock.header.frame_id = "base_link";
    fake_dock.pose.position.x = 1.0;
    fake_dock.pose.orientation.w = 1.0;
    tf2_listen_.transformPose("map", fake_dock, idp);
  }

  void run() {
    update_init_dock(init_dock_pose_);
    perception_->start(init_dock_pose_);

    ros::Rate rate(50);  // 20ms
    while (ros::ok()) {
      geometry_msgs::PoseStamped dock_pose;

      if (!perception_->getPose(dock_pose, "map") && !found_dock_) {
        ROS_INFO_THROTTLE(1.0, "Still finding dock...");
        update_init_dock(init_dock_pose_);
        perception_->start(init_dock_pose_);
      } else {
        found_dock_ = true;

        lidar_auto_docking::Initdock dock_msg;
        dock_msg.x = dock_pose.pose.position.x;
        dock_msg.y = dock_pose.pose.position.y;
        dock_msg.z = dock_pose.pose.orientation.z;
        dock_msg.w = dock_pose.pose.orientation.w;
        publisher_.publish(dock_msg);

        // Broadcast dock TF for visualisation
        geometry_msgs::TransformStamped ts;
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "map";
        ts.child_frame_id = "dock";
        ts.transform.translation.x = dock_pose.pose.position.x;
        ts.transform.translation.y = dock_pose.pose.position.y;
        ts.transform.translation.z = 0.0;
        ts.transform.rotation.x = 0;
        ts.transform.rotation.y = 0;
        ts.transform.rotation.z = dock_pose.pose.orientation.z;
        ts.transform.rotation.w = dock_pose.pose.orientation.w;
        tbr_->sendTransform(ts);
      }

      ros::spinOnce();
      rate.sleep();
    }
  }

 private:
  ros::NodeHandle nh_;
  std::shared_ptr<DockPerception> perception_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tbr_;
  ros::Publisher publisher_;
  ros::Subscriber joy_sub_;
  tf2_listener tf2_listen_;

  geometry_msgs::PoseStamped dock_pose_;
  geometry_msgs::PoseStamped init_dock_pose_;
  bool found_dock_;
  int reset_goal_button_;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "dock_coordinates");
  ros::NodeHandle nh;

  DockCoordinates node(nh);
  node.init_objects();
  node.run();

  return 0;
}
