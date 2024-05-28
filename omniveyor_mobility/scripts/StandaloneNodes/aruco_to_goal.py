#!/usr/bin/env python

from geometry_msgs.msg import Pose, Point, Quaternion
import rospy
from std_msgs.msg import Empty
from aruco_msgs.msg import MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import termios
import tty
import sys
import select
import numpy as np
import tf.transformations


class ArucoToGoal:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        self.aruco_markers = rospy.Subscriber(
            "/aruco_marker_publisher/markers", MarkerArray, self.aruco_cb
        )
        self.goal_trigger = rospy.Subscriber(
            "/aruco_goal_send", Empty, self.goal_trig_cb
        )
        self.goal_pub = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # IDs are arcuo pattern 26 & 582
        self.aruco_markers = [26, 582]
        self.offset_dist = float(rospy.get_param("~offset_dist"))
        self.desired_goal = MoveBaseGoal()
        self.goal_pub.wait_for_server()
        print("Aruco To Goal initialized")

    def aruco_cb(self, msg):
        for marker in msg.markers:
            if marker.id in self.aruco_markers:
                goal = MoveBaseGoal()
                goal_target = goal.target_pose
                # goal_target.header.frame_id = "base_link"
                goal_target.header.stamp = rospy.Time.now()
                goal_target.header.frame_id = "map"
                goal_target.pose = self.correct_pose(marker)
                self.desired_goal = goal

    def correct_pose(self, marker):
        pose_obj = marker.pose.pose
        marker_pos = [pose_obj.position.x, pose_obj.position.y, pose_obj.position.z]
        orientation = [
            pose_obj.orientation.w,
            pose_obj.orientation.x,
            pose_obj.orientation.y,
            pose_obj.orientation.z,
        ]

        # offset position in z direction - in front b/c that how aruco marker work
        rot_matrix = tf.transformations.quaternion_matrix(orientation)
        offset = np.array([0, 0, self.offset_dist])
        offset = np.dot(rot_matrix, offset)
        goal_pos = marker_pos + offset

        # rotate quaternion to have AGV face the aruco marker
        vector = marker_pos - goal_pos
        vector /= np.linalg.norm(vector)
        angle = np.arctan2(vector[1], vector[0])
        goal_orient = tf.transformations.quaternion_from_euler(0, 0, angle)
        goal_orient = goal_orient[1:] + [goal_orient[0]]

        goal = Pose(Point(*goal_pos), Quaternion(*goal_orient))
        return goal

    def goal_trig_cb(self, _):
        rospy.loginfo("Sent goal: \n\r" + str(self.desired_goal))
        self.goal_pub.send_goal_and_wait(self.desired_goal)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        res, _, _ = select.select([sys.stdin], [], [], 1.0)
        key = "" if not res else sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def main(self):
        while not rospy.is_shutdown():
            key = self.getKey()
            if key == "D":
                rospy.loginfo("Sent goal: \n\r" + str(self.desired_goal))
                self.goal_pub.send_goal_and_wait(self.desired_goal)
            elif not key:
                pass
            elif key == "\x03":
                break
            else:
                if key != "D":
                    print("Please wait until AGV is ready")


if __name__ == "__main__":
    try:
        rospy.init_node("aruco_to_goal", anonymous=True)
        ag = ArucoToGoal()
        ag.main()
    except rospy.ROSInterruptException:
        sys.exit()
