#!/usr/bin/env python3
"""
ROS1 port of nav2_dock.py
Replaces: nav2_msgs NavigateToPose → move_base MoveBaseAction
          rclpy_action              → actionlib
          QoSProfile                → standard rospy subscriber
"""
import rospy
import json
import actionlib
from std_msgs.msg import Int32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from lidar_auto_docking.msg import DockAction, DockGoal, UndockAction, UndockGoal


class undocking_client:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('Undock', UndockAction)
        self.goal_status = False
        self.goal_accept_status = False
        self.failure_flag = False

    def send_goal(self):
        rospy.loginfo("Waiting for Undock server...")
        self.client.wait_for_server()
        goal = UndockGoal()
        goal.rotate_in_place = True
        rospy.loginfo("Sending undock goal")
        self.client.send_goal(goal,
                              done_cb=self.done_cb,
                              active_cb=self.active_cb)

    def active_cb(self):
        self.goal_accept_status = True

    def done_cb(self, state, result):
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Undocking succeeded!")
            self.goal_status = True
        else:
            rospy.logwarn("Undocking failed!")
            self.goal_status = False
            self.failure_flag = True

    def get_status(self):
        return {"gs": self.goal_status,
                "gas": self.goal_accept_status,
                "f_flag": self.failure_flag}

    def reset_status(self):
        self.goal_status = False
        self.goal_accept_status = False
        self.failure_flag = False


class docking_client:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('Dock', DockAction)
        self.goal_status = False
        self.goal_accept_status = False
        self.failure_flag = False

    def send_goal(self, dock_pose):
        rospy.loginfo("Waiting for Dock server...")
        self.client.wait_for_server()
        goal = DockGoal()
        goal.dock_pose.header.frame_id = "map"
        goal.dock_pose.header.stamp = rospy.Time.now()
        goal.dock_pose.pose.position.x = dock_pose["x"]
        goal.dock_pose.pose.position.y = dock_pose["y"]
        goal.dock_pose.pose.orientation.z = dock_pose["z"]
        goal.dock_pose.pose.orientation.w = dock_pose["w"]
        rospy.loginfo("Sending dock goal")
        self.client.send_goal(goal,
                              done_cb=self.done_cb,
                              active_cb=self.active_cb)

    def active_cb(self):
        self.goal_accept_status = True

    def done_cb(self, state, result):
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Docking succeeded!")
            self.goal_status = True
        else:
            rospy.logwarn("Docking failed!")
            self.goal_status = False
            self.failure_flag = True

    def get_status(self):
        return {"gs": self.goal_status,
                "gas": self.goal_accept_status,
                "f_flag": self.failure_flag}

    def reset_status(self):
        self.goal_status = False
        self.goal_accept_status = False
        self.failure_flag = False


class goto_pose:
    def __init__(self):
        # ROS1: move_base replaces nav2 NavigateToPose
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal_status = False
        self.goal_accept_status = False
        self.failure_flag = False

    def send_goal(self, goal_pose):
        rospy.loginfo("Waiting for move_base server...")
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_pose["bx"]
        goal.target_pose.pose.position.y = goal_pose["by"]
        goal.target_pose.pose.orientation.z = goal_pose["bz"]
        goal.target_pose.pose.orientation.w = goal_pose["bw"]
        rospy.loginfo("Sending navigation goal")
        self.client.send_goal(goal,
                              done_cb=self.done_cb,
                              active_cb=self.active_cb)

    def active_cb(self):
        self.goal_accept_status = True
        rospy.loginfo("Navigation goal accepted")

    def done_cb(self, state, result):
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation succeeded!")
            self.goal_status = True
        else:
            rospy.logwarn("Navigation failed!")
            self.goal_status = False
            self.failure_flag = True

    def get_status(self):
        return {"gs": self.goal_status,
                "gas": self.goal_accept_status,
                "f_flag": self.failure_flag}

    def reset_status(self):
        self.goal_status = False
        self.goal_accept_status = False
        self.failure_flag = False


class MainLogic:
    def __init__(self):
        rospy.init_node('dock_logic')

        self.dock_cmd = 0
        self.dock_stat = 0

        dock_file_path = rospy.get_param('~load_file_path')
        with open(dock_file_path) as f:
            self.initial_poses = json.load(f)

        self.goto_pose_ = goto_pose()
        self.docking_client_ = docking_client()
        self.undocking_client_ = undocking_client()

        self.cmd_sub = rospy.Subscriber('dock_cmd', Int32, self.update_cmd)
        self.status_pub = rospy.Publisher('dock_status', Int32, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timed_callback)

    def update_cmd(self, msg):
        self.dock_cmd = msg.data

    def timed_callback(self, event):
        nav_status   = self.goto_pose_.get_status()
        dock_status  = self.docking_client_.get_status()
        undock_status = self.undocking_client_.get_status()

        # Failure check
        if (nav_status["f_flag"] or dock_status["f_flag"] or
                undock_status["f_flag"]) and self.dock_stat != 10:
            rospy.logwarn("Docking failure!")
            self.dock_stat = 10

        # State machine — mirrors original nav2_dock.py logic
        elif self.dock_cmd == 1 and self.dock_stat == 0:
            self.goto_pose_.send_goal(self.initial_poses)
            self.dock_stat = 1

        elif (self.dock_stat == 1 and
              nav_status["gs"] and nav_status["gas"]):
            rospy.loginfo("Arrived at pre-dock pose, starting dock sequence")
            self.docking_client_.send_goal(self.initial_poses)
            self.dock_stat = 2

        elif (self.dock_stat == 2 and
              dock_status["gs"] and dock_status["gas"]):
            rospy.loginfo("Docked! Waiting for undock command...")
            self.dock_stat = 3

        elif self.dock_stat == 3 and self.dock_cmd == 2:
            rospy.loginfo("Undocking...")
            self.undocking_client_.send_goal()
            self.dock_stat = 4

        elif (self.dock_stat == 4 and
              undock_status["gs"] and undock_status["gas"]):
            rospy.loginfo("Docking sequence complete, resetting")
            self.dock_stat = 0
            self.undocking_client_.reset_status()
            self.docking_client_.reset_status()
            self.goto_pose_.reset_status()

        elif self.dock_stat == 10 and self.dock_cmd == 10:
            rospy.loginfo("Resetting docking state after failure")
            self.dock_stat = 0
            self.undocking_client_.reset_status()
            self.docking_client_.reset_status()
            self.goto_pose_.reset_status()

        msg = Int32()
        msg.data = self.dock_stat
        self.status_pub.publish(msg)


def main():
    node = MainLogic()
    rospy.spin()


if __name__ == "__main__":
    main()
