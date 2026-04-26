#!/usr/bin/env python3
import rospy
import json
import actionlib
from lidar_auto_docking.msg import DockAction, DockGoal
from geometry_msgs.msg import PoseStamped


class docking_client:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('Dock', DockAction)
        self.goal_status = False

    def send_goal(self, dock_pose):
        rospy.loginfo("Waiting for Dock action server...")
        self.client.wait_for_server()

        goal = DockGoal()
        goal.dock_pose.header.frame_id = "map"
        goal.dock_pose.header.stamp = rospy.Time.now()
        goal.dock_pose.pose.position.x = dock_pose["x"]
        goal.dock_pose.pose.position.y = dock_pose["y"]
        goal.dock_pose.pose.orientation.z = dock_pose["z"]
        goal.dock_pose.pose.orientation.w = dock_pose["w"]

        rospy.loginfo("Sending dock goal...")
        self.client.send_goal(goal,
                              done_cb=self.done_cb,
                              active_cb=self.active_cb)

    def active_cb(self):
        rospy.loginfo("Dock goal accepted")

    def done_cb(self, state, result):
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Docking succeeded!")
            self.goal_status = True
        else:
            rospy.logwarn("Docking failed with state: %d", state)
            self.goal_status = False

    def get_status(self):
        return {"gs": self.goal_status}

    def reset_status(self):
        self.goal_status = False


def main():
    rospy.init_node('dock_robot')

    dock_file_path = rospy.get_param('~load_file_path')
    with open(dock_file_path) as f:
        initial_dock_pose = json.load(f)

    client = docking_client()
    client.send_goal(initial_dock_pose)

    rospy.spin()


if __name__ == "__main__":
    main()