#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal

class NavClient:
    def __init__(self):
        rospy.loginfo('Waiting for move_base action server')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo('move_base client ready')

    def send_goal(self, goal):
        rospy.loginfo("Sending navigation goal %s" % (str(goal)))
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        rospy.loginfo("Navigating done: %s" % (str(result)))
        return result

if __name__ == '__main__':
    try:
        rospy.init_node('random_nav')
        rospy.loginfo('nav demo started')

        client = NavClient()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()

        # back up one meter
        goal.target_pose.pose.position.x = -1.0
        goal.target_pose.pose.orientation.w = 1.0

        client.send_goal(goal)

    except rospy.ROSInterruptException:
        pass
    rospy.loginfo('nav demo done')
