#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import GoalStatusArray
import numpy as np

class TrajRunner():
    def __init__(self):
        self.fname = rospy.get_param("~traj_file")
        self.traj = np.loadtext(self.fname, delimiter=' ')
        print(self.traj)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.goal_status = rospy.Subscriber('/move_base/status', )
        
    
    def main(self):
        rospy.spin()


        
if __name__=="__main__":
    rospy.init_node('trajectory_runner', anonymous=True)
    tw = TrajRunner()
    tw.main()