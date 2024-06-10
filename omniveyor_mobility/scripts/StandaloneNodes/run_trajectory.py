#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from omniveyor_common.msg import GoalTriggerAction
import numpy as np
import actionlib


class TrajRunner:
    goal_num_to_aruco_num = {0: 26, 1: 528, 2: 123}

    def __init__(self):
        self.fname = str(rospy.get_param("~traj_file"))
        self.traj = np.loadtxt(self.fname, delimiter=" ")
        self.goal_status = -1
        self.goal_pub = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.goal_trig_aruco = actionlib.SimpleActionClient(
            "/goal_aruco_trigger", GoalTriggerAction
        )
        rospy.Subscriber("/goal_send_trigger", Int8, self.goal_trig_cb, queue_size=1)
        rospy.logdebug("TrajRunner Waiting for servers\r")
        self.goal_pub.wait_for_server()
        self.goal_trig_aruco.wait_for_server()
        rospy.logdebug("Action servers ready - TrajRunner Ready!")

    def goal_trig_cb(self, goal_num):
        if len(self.traj) - 1 < goal_num:
            rospy.logerr(
                f"Recieved invalid goal #{goal_num}, only have {len(self.traj)} goals"
            )
            return
        goal_pose = self.traj[goal_num]
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose = Pose(Point(*goal_pose[:3]), Quaternion(*goal_pose[3:]))
        rospy.loginfo(f"Sent goal #{goal_num} out: \r" + str(goal))
        self.goal_pub.send_goal_and_wait(goal)
        if self.goal_pub.get_state() == actionlib.TerminalState.SUCCEEDED:
            self.goal_trig_aruco_func(goal_num)
        else:
            rospy.logerr("Failed to reach goal: \r" + str(self.goal_pub.get_result()))

    def goal_trig_aruco_func(self, goal_num):
        self.goal_trig_aruco.send_goal_and_wait(
            Int8(self.goal_num_to_aruco_num[goal_num])
        )
        action_state = self.goal_trig_aruco.get_state()
        if action_state == actionlib.TerminalState.SUCCEEDED:
            rospy.loginfo(
                "Succesfully reached desired trajectory goal and lined up with aruco marker\r"
            )
        else:
            rospy.logerr(
                "Failed to reach goal: \r"
                + str(self.goal_trig_aruco.get_result())
                + "\r\nState was: "
                + str(action_state)
            )

    def main(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        rospy.init_node("trajectory_runner", anonymous=True)
        tw = TrajRunner()
        tw.main()
    except rospy.ROSInterruptException as e:
        print(e)
