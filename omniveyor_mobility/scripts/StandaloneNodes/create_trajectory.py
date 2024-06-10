#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class TrajWriter:
    def __init__(self):
        self.traj = []
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
        rospy.Subscriber("/goal_send_trigger", Int8, self.pos_goal_cb)
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
        rospy.loginfo("Recorded: %s as goal #%d\r" % (str(new_goal), len(self.traj)))
        self.traj.append(new_goal)

    def pos_goal_cb(self, msg):
        current_position = rospy.wait_for_message("map_pose", PoseWithCovarianceStamped)
        assert type(current_position) is PoseWithCovarianceStamped
        new_goal = [
            round(value, 5)
            for value in [
                current_position.pose.pose.position.x,
                current_position.pose.pose.position.y,
                current_position.pose.pose.position.z,
                current_position.pose.pose.orientation.x,
                current_position.pose.pose.orientation.y,
                current_position.pose.pose.orientation.z,
                current_position.pose.pose.orientation.w,
            ]
        ]
        # Add to desired location or fail
        if len(self.traj) >= msg.data + 1:
            self.traj[msg.data] = new_goal
            rospy.loginfo("Recorded: %s as goal #%d\r" % (str(new_goal), msg.data))
        elif len(self.traj) == msg.data:
            self.traj.append(new_goal)
            rospy.loginfo("Recorded: %s as goal #%d\r" % (str(new_goal), msg.data))
        else:
            missing_goals = str(list(range(len(self.traj), msg.data)))
            rospy.logwarn(f"Missing goals {missing_goals}, Not saving desired goal")

    def main(self):
        rospy.spin()

    def on_kill(self):
        np.savetxt(self.fname, np.array(self.traj), delimiter=" ")
        rospy.loginfo(f"Saved goals in {self.fname}")


if __name__ == "__main__":
    rospy.init_node("trajectory_writer", anonymous=True)
    tw = TrajWriter()
    tw.main()
