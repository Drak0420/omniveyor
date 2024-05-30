#!/usr/bin/env python

import actionlib
import numpy as np
import rospy
import tf.transformations

from scipy.spatial.transform import Rotation as R
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from omniveyor_mobility.msg import GoalTriggerAction
from std_msgs.msg import Empty


def offset_pose(pose_obj, offset_dist):
    marker_pos = [pose_obj.position.x, pose_obj.position.y, 0]
    orientation = [
        pose_obj.orientation.x,
        pose_obj.orientation.y,
        pose_obj.orientation.z,
        pose_obj.orientation.w,
    ]

    # offset position in z direction - in front b/c that how aruco marker work
    rotation = R.from_quat(orientation)
    rot_pos = rotation.apply([0, 0, offset_dist])
    goal_pos = [rot_pos[0] + marker_pos[0], rot_pos[1] + marker_pos[1], 0]

    # rotate quaternion to have AGV face the aruco marker
    vector = np.array(marker_pos) - np.array(goal_pos)
    vector = np.divide(vector, np.linalg.norm(vector))
    angle = np.arctan2(vector[1], vector[0])
    goal_orient = tf.transformations.quaternion_from_euler(0, 0, angle)

    goal = Pose(Point(*goal_pos), Quaternion(*goal_orient))
    return goal


class ArucoToGoal:
    def __init__(self):
        # IDs are arcuo pattern 26 & 582
        self.marker_ids = [26, 582]
        self.offset_dist = float(rospy.get_param("~offset_dist"))
        self.desired_goal = MoveBaseGoal()

        rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, self.aruco_cb)
        self.goal_trig_recieve = actionlib.SimpleActionServer(
            "/goal_aruco_trigger", GoalTriggerAction, self.goal_trig_cb, False
        )
        self.goal_trig_recieve.start()
        rospy.Subscriber("/aruco_goal_send", Empty, self.goal_trig_cb)
        self.goal_pub = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.goal_pub.wait_for_server()
        rospy.loginfo("Aruco To Goal initialized")

    def aruco_cb(self, msg):
        for marker in msg.markers:
            if int(marker.id) in self.marker_ids:
                goal = MoveBaseGoal()
                goal_target = goal.target_pose
                goal_target.header.stamp = rospy.Time.now()
                goal_target.header.frame_id = "map"
                goal_target.pose = offset_pose(marker.pose.pose, self.offset_dist)
                self.desired_goal = goal

    def goal_trig_cb(self, _):
        rospy.loginfo("Sent goal: \n\r" + str(self.desired_goal))
        self.goal_pub.send_goal_and_wait(self.desired_goal)
        goal_result = self.goal_pub.get_result()
        assert goal_result is not None
        if self.goal_pub.get_state() == actionlib.TerminalState.SUCCEEDED:
            print(str(goal_result))
            print(goal_result)
            self.goal_trig_recieve.set_succeeded(goal_result.status)
        else:
            self.goal_trig_recieve.set_aborted(goal_result.status)

    def main(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        rospy.init_node("aruco_to_goal", anonymous=True)
        ag = ArucoToGoal()
        ag.main()
    except rospy.ROSInterruptException:
        print("Keyboard interrupt, exited aruco_to_goal")
