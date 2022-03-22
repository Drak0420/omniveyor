#!/usr/bin/python

import os
import sys
import rospy
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)
from ReFrESH_ROS import ReFrESH_Module
from ReFrESH_ROS_utils import Ftype, RingBuffer
from utils import tortuosity
import dynamic_reconfigure.client
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseGoal
from omniveyor_common.msg import LowLevelNavigationGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
import numpy as np

""" Manipulate Move Base planners """
class moveBaseModule(ReFrESH_Module):
    def __init__(self, name="moveBaseMotion", priority=97, preemptive=True, bgp="", blp=""):
        super().__init__(name, priority=priority, preemptive=preemptive)
        # Tortuosity of path, time used, final error
        self.performanceMetrics = [0.0, 0.0]
        self.tortuosityTol = 1.5
        # Resource metric: not-availability
        self.resourceMetrics = [0.5]
        self.moveBaseGoal = None
        self.abortedGoal = None         # aborted goal, set when the module is turned off without reaching the last goal.
        self.setComponentProperties('EX', Ftype.ACTION_CLI, 'move_base', self.feedbackCb, mType=MoveBaseGoal, 
                                    kwargs={'active_cb': self.activeCb, 'done_cb': self.doneCb, 'goal': self.moveBaseGoal,
                                            'prelaunch_cb': self.prelaunch, 'availTimeout': 1.0},
                                    ind=0)
        self.setComponentProperties('EV', Ftype.TIMER, exec=self.evaluator, kwargs={'freq': 3.0})
        self.setComponentProperties('ES', Ftype.CALLABLE, exec=self.estimator)
        self.historyPoses = RingBuffer(50)
        # base global planner
        self.bgp = bgp
        # base local planner
        self.blp = blp
    
    def syncGoal(self):
        self.moveBaseGoal = self.managerHandle.moveBaseGoal
        if self.managerHandle.moduleIsOn(self):
            # already an active module. Reusing it.
            exHandle = self.managerHandle.getEXhandle(self)
            # cancel ongoing goal and submit new goal
            exHandle.cancel_all_goals()
            exHandle.send_goal(self.moveBaseGoal, self.doneCb, self.activeCb, self.feedbackCb)
        else:
            self.setComponentProperties('EX', Ftype.ACTION_CLI, 'move_base', self.feedbackCb, mType=MoveBaseGoal, 
                                kwargs={'active_cb': self.activeCb, 'done_cb': self.doneCb, 'goal': self.moveBaseGoal,
                                        'prelaunch_cb': self.prelaunch, 'availTimeout': 1.0},
                                ind=0)
    
    def setAbortedGoal(self, goal):
        self.abortedGoal = goal

    def prelaunch(self):
        if not self.moveBaseGoal:
            raise RuntimeError('The module has not initialized yet!')
        # use dynamic reconfiguration to change the planner type to desired ones.
        drc = dynamic_reconfigure.client.Client("move_base", timeout=3.0, config_callback=None)
        if drc:
            drc.get_configuration()
            drc.update_configuration({'base_gkibal_planner': self.bgp, 'base_local_planner': self.blp})
        else:
            raise RuntimeError('Move Base parameter reconfiguration server is inactive!')
    
    def feedbackCb(self, feedback):
        # synchronize feedback to this class, and to the manager class.
        pose = feedback.base_position
        # use raw quaternion since it's continuous.
        self.historyPoses.append([pose.header.stamp.to_secs(), pose.pose.position.x, pose.pose.position.y, 
                            pose.pose.orientation.z, pose.pose.orientation.w])
        self.managerHandle.setFeedback(feedback)
        pass
    
    def activeCb(self):
        # notify the manager class that the goal has been accepted.
        pass
    
    def doneCb(self, status, result):
        # ring the bell on the manager about the result.
        # movebaseresult is just an empty message.
        if status == GoalStatus.SUCCEEDED:
            # this goal is achieved
            self.managerHandle.setResult(status, final=True)
        else:
            # 2: canceled after started. Need manager to determine the origin of cancellation.
            # 4: aborted. Need manager to determine status of other module
            # 5: rejected. Need manager to determine status of other module
            # 8: canceled before start. Need manager to determine the origin of cancellation.
            if status==GoalStatus.ABORTED or status==GoalStatus.REJECTED:
                # TODO: does the result contain a pose where it failed?
                # If not, need another way to remember the aborted pose (maybe last feedback?)
                self.abortedGoal = self.managerHandle.currentGoal
            self.managerHandle.setResult(status, final=False)
        pass

    def evaluator(self, event):
        # /move_base/make_plan service available, distance to goal, time elapsed, turtuosity of path
        rowVect = self.historyPoses.get() 
        self.performanceMetrics[0] = tortuosity(rowVect)/self.tortuosityTol
        timeNow = rospy.Time.now()
        if timeNow < self.managerHandle.currentGoal.expiration:
            # still a valid task.
            self.performanceMetrics[1] = np.max((self.remaining_distance - self.managerHandle.currentGoal.tolerance)
                                                / (self.managerHandle.currentGoal.expiration - timeNow).to_secs
                                                / self.speedLimit, 0.)
        else:
            # the task has expired. Set final status.
            self.performanceMetrics[1] = 1.0
            self.managerHandle.setResult(GoalStatus.ABORTED, final=True)
        # time_elapsed resets if distance_to_goal decreases every, say, 0.5m?
        # if timed out && distance > tolerance set failed flag.
        # if failed retries ++

    def estimator(self):
        # this happens before any module has run, or after another planner module has failed.
        # assume MoveBase is running, no active plans
        # configure the planner to what this module will have
        self.prelaunch()
        if self.resourceMetrics[0]>=1.0:
            # TODO: this module has failed before. Check if the goal is in a new position.
            distance = np.array(self.abortedGoal.target_pose.pose.position) - np.array(self.managerHandle.currentGoal.target_pose.pose.position)
            goalPosTol = 0.5*(self.abortedGoal.target_pose.pose.covariance[0]+self.abortedGoal.target_pose.pose.covariance[7])
            # the base can rotate in place, so goal pose is less important in determining success.
            new_goal_pos = np.dot(distance, distance) > 9.*goalPosTol*goalPosTol
            # if the goal is at a new position, reset retries = 0, availability to 1 (metric=0)
            if new_goal_pos:
                self.resourceMetrics[0] = 0.0
                self.retry = 0
                if self.performanceMetrics[0] >= 1.0:
                    self.performanceMetrics[0] = 0.0    # Replace with nominal performance instead of 0?
                if self.performanceMetrics[1] >= 1.0:
                    self.performanceMetrics[1] = 0.0
                if self.performanceMetrics[2] >= 1.0:
                    self.performanceMetrics[2] = 0.0
            # handle retries
            # if retries < max_retry set availability to 1 (metric=0)
            # TODO: if retries >= max_retry and || position - last_failed_position || < 1m set availability to 0 (metric=1)
            elif self.retry < self.max_retry:
                self.retry += 1
                self.resourceMetrics[0] = 0.0
        # this module has not previously failed. test it.
        if not self.resourceMetrics[0]:
            # /move_base/make_plan service available and generates a non-empty plan
            rospy.wait_for_service('/move_base/make_plan', timeout=3.0)
            start = PoseStamped()
            startTmp = self.managerHandle.getWorldPose()
            start.header = startTmp.header
            start.pose = startTmp.pose.pose
            goal = PoseStamped()
            goal.header = self.managerHandle.currentGoal.header
            goal.pose = self.managerHandle.currentGoal.target_pose.pose
            try:
                planSrv = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
                plan = planSrv(start, goal, (self.managerHandle.currentGoal.target_pose.covariance[0]+\
                                            self.managerHandle.currentGoal.target_pose.covariance[7])*0.5)
                self.resourceMetrics[0] = 0.0 if len(plan.poses) else 1.0
            except rospy.ServiceException as e:
                print("ERROR: Service call failed: %s"%e)
                self.resourceMetrics[0] = 1.0
    
"""TODO: Follows a path as-is with PID controller"""
class viaPointFollowerModule(ReFrESH_Module):
    def __init__(self, name="moveBaseMotion", priority=96, preemptive=True, pathTopic="raw_path"):
        super().__init__(name, priority=priority, preemptive=preemptive)
        # Tortuosity of path, time used, final error
        self.performanceMetrics = [0.0, 0.0, 0.0]
        # Resource metric: nearest obstacle threshold, CPU utilization, mem Utilization
        self.resourceMetrics = [0.5, 0.0, 0.0]
        # use two EX threads instead?
        self.setComponentProperties('EX', Ftype.SUBSCRIBER, pathTopic, self.getPath, mType=Path)
        self.setComponentProperties('EV', Ftype.TIMER, exec=self.evaluator, kwargs={'freq': 3.0})
        self.setComponentProperties('ES', Ftype.TIMER, exec=self.estimator, kwargs={'freq': 3.0})

    def getPath(self, msg):
        # pid controller to next point in the path
        pass

    def evaluator(self, event):
        # footprint intrusion
        pass

    def estimator(self, event):
        # is path clear, or, /move_base/make_plan service available and generates a non-empty plan?
        pass