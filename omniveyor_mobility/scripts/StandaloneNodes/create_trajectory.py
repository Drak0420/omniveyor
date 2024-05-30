#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped


class TrajWriter:
    def __init__(self):
        self.traj = []
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
        self.fname = str(rospy.get_param("~traj_file"))
        rospy.on_shutdown(self.on_kill)

    def goal_cb(self, msg):
        new_goal = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        rospy.loginfo("Recorded: %s as goal #%d\n\r" % (str(new_goal), len(self.traj)))
        self.traj.append(new_goal)

    def main(self):
        rospy.spin()

    def on_kill(self):
        np.savetxt(self.fname, np.array(self.traj), delimiter=" ")


if __name__ == "__main__":
    rospy.init_node("trajectory_writer", anonymous=True)
    tw = TrajWriter()
    tw.main()
