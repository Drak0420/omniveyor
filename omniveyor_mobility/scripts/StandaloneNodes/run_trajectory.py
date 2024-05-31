#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from omniveyor_common.msg import GoalTriggerAction, GoalTriggerGoal
import numpy as np
import actionlib


class TrajRunner:
    def __init__(self):
        self.fname = str(rospy.get_param("~traj_file"))
        self.traj = np.loadtxt(self.fname, delimiter=" ")
        self.trajNum = 0
        self.goal_status = -1
        self.goal_pub = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.goal_trig_aruco = actionlib.SimpleActionClient(
            "/goal_aruco_trigger", GoalTriggerAction
        )
        rospy.Subscriber("/goal_send_trigger", Empty, self.goal_trig_cb, queue_size=1)
        rospy.logdebug("TrajRunner Waiting for servers\r\n")
        self.goal_pub.wait_for_server()
        self.goal_trig_aruco.wait_for_server()
        rospy.logdebug("Action servers ready - TrajRunner Ready!")

    def goal_trig_cb(self, _):
        # if not self.goal_pub.wait_for_result(rospy.Duration(1)):
        #    rospy.logwarn("Recieved goal trigger while running already\r\n")
        #    return
        goal_pose = self.traj[self.trajNum]
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose = Pose(Point(*goal_pose[:3]), Quaternion(*goal_pose[3:]))
        rospy.loginfo(f"Sent goal #{self.trajNum} out: \n\r" + str(goal))
        self.goal_pub.send_goal_and_wait(goal)
        if self.goal_pub.get_state() == actionlib.TerminalState.SUCCEEDED:
            self.trajNum = self.trajNum + 1
            if self.trajNum >= len(self.traj):
                self.trajNum = 0
            self.goal_trig_aruco_func()
        else:
            rospy.logerr("Failed to reach goal: \r\n" + str(self.goal_pub.get_result()))

    def goal_trig_aruco_func(self):
        self.goal_trig_aruco.send_goal_and_wait(GoalTriggerGoal())
        action_state = self.goal_trig_aruco.get_state()
        if action_state == actionlib.TerminalState.SUCCEEDED:
            rospy.loginfo(
                "Succesfully reached desired trajectory goal and lined up with aruco marker\n\r"
            )
        else:
            rospy.logerr(
                "Failed to reach goal: \r\n"
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
