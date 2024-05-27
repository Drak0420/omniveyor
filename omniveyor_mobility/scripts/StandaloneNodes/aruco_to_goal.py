#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from aruco_msgs.msg import MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion
import math
import actionlib
import termios
import tty
import sys
import select


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
                offsets = self.get_offsets(marker)
                goal_target.header.frame_id = "map"
                goal_target.pose.position.x = offsets[0]
                goal_target.pose.position.y = offsets[1]
                goal_target.pose.position.z = 0.0
                goal_target.pose.orientation.x = 0.0
                goal_target.pose.orientation.y = 0.0
                goal_target.pose.orientation.z = marker.pose.pose.orientation.y
                goal_target.pose.orientation.w = 0.0
                self.desired_goal = goal

    def get_offsets(self, marker):
        pose_obj = marker.pose.pose
        orientation = pose_obj.orientation
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        x_offset = self.offset_dist * math.cos(pitch) + pose_obj.position.x
        y_offset = self.offset_dist * math.sin(pitch) + pose_obj.position.y
        return [x_offset, y_offset]

    def goal_trig_cb(self, _):
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
