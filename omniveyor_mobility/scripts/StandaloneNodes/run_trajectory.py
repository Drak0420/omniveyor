#!/usr/bin/env python

import sys
import select
import termios
import tty
import rospy
from time import time
from actionlib_msgs.msg import GoalID, GoalStatusArray
from actionlib_msgs.msg import MoveBaseActionGoal
import numpy as np


class TrajRunner:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        self.fname = rospy.get_param("~traj_file")
        self.traj = np.loadtxt(self.fname, delimiter=" ")
        self.trajNum = 0
        self.goal_status = -1
        self.prev_time = time()
        print(self.traj)
        self.goal_pub = rospy.Publisher(
            "/move_base/goal", MoveBaseActionGoal, queue_size=1
        )
        self.cancel_goal_pub = rospy.Publisher(
            "/move_base/cancel", GoalID, queue_size=1
        )
        self.goal_status = rospy.Subscriber(
            "/move_base/status", GoalStatusArray, self.status_cb
        )

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        res, _, _ = select.select([sys.stdin], [], [], 1.0)
        key = "" if not res else sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def status_cb(self, msg):
        try:
            self.goal_status = msg.status_list[0].status
            time_elapsed = time() - self.prev_time
            if time_elapsed > 2:
                print("Current status: " + msg.status_list[0].text)
                self.prev_time = time()
        except AttributeError as e:
            print(e)
            if self.goal_status == -1:
                self.goal_status = 3
                print("Waiting for first goal")

    def main(self):
        while not rospy.is_shutdown():
            key = self.getKey()
            if self.goal_status != -1 and key == "D":
                print(f"Sent goal #{self.trajNum} out")
                goal_pose = self.traj[self.trajNum]
                msg = MoveBaseActionGoal()
                msg.goal.target_pose.header.stamp = rospy.Time.now()
                msg.goal.target_pose.header.frame_id = "map"
                msg.goal.target_pose.pose.position.x = goal_pose[0]
                msg.goal.target_pose.pose.position.y = goal_pose[1]
                msg.goal.target_pose.pose.position.z = goal_pose[2]
                msg.goal.target_pose.pose.orientation.x = goal_pose[3]
                msg.goal.target_pose.pose.orientation.y = goal_pose[4]
                msg.goal.target_pose.pose.orientation.z = goal_pose[5]
                msg.goal.target_pose.pose.orientation.w = goal_pose[6]
                self.goal_pub.publish(msg)
                self.goal_pub.publish(msg)
                self.trajNum = self.trajNum + 1
                if self.trajNum >= len(self.traj):
                    self.trajNum = 0
            elif key == "C":  # Cancels current goal
                self.cancel_goal_pub.publish(GoalID())
            elif not key:
                pass
            elif key == "\x03":
                break
            else:
                if key != "D":
                    print("Please wait until AGV is ready")


if __name__ == "__main__":
    rospy.init_node("trajectory_runner", anonymous=True)
    tw = TrajRunner()
    tw.main()
