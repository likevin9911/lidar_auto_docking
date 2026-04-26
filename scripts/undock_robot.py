#!/usr/bin/env python3
import rospy
import actionlib
from lidar_auto_docking.msg import UndockAction, UndockGoal


class undocking_client:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('Undock', UndockAction)
        self.goal_status = False

    def send_goal(self):
        rospy.loginfo("Waiting for Undock action server...")
        self.client.wait_for_server()

        goal = UndockGoal()
        goal.rotate_in_place = True

        rospy.loginfo("Sending undock goal...")
        self.client.send_goal(goal,
                              done_cb=self.done_cb,
                              active_cb=self.active_cb)

    def active_cb(self):
        rospy.loginfo("Undock goal accepted")

    def done_cb(self, state, result):
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Undocking succeeded! Undocked: %s", str(result.undocked))
            self.goal_status = True
        else:
            rospy.logwarn("Undocking failed with state: %d", state)
            self.goal_status = False

    def get_status(self):
        return {"gs": self.goal_status}

    def reset_status(self):
        self.goal_status = False


def main():
    rospy.init_node('undock_robot')

    client = undocking_client()
    client.send_goal()

    rospy.spin()


if __name__ == "__main__":
    main()