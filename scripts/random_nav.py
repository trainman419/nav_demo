#!/usr/bin/env python

import rospy
import actionlib
import smach
import smach_ros
import math
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from kobuki_msgs.msg import AutoDockingAction,AutoDockingGoal,PowerSystemEvent
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Twist

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

class DockClient:
    def __init__(self):
        rospy.loginfo('Waiting for auto docking action server')
        self.client = actionlib.SimpleActionClient('dock_drive_action',
                        AutoDockingAction)
        self.client.wait_for_server()
        rospy.loginfo('auto docking client ready')

    def dock(self):
        rospy.loginfo('docking')
        goal = AutoDockingGoal()
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        rospy.loginfo("docking done: %s" % (str(result)))
        return result

    def undock(self):
        rospy.loginfo('undocking (TODO)')

# state machine:
# top-level states:
#  - charging
#  - undocking
#  - docking
#  - navigating
#
# transitions:
# - charged: charging->undocking
# - succeeded: undocking->navigating
# - succeeded: docking->charging
# - battery low: navigating->docking

class Charging(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['charged'])
        self.charged = True
        self.sub = rospy.Subscriber('/mobile_base/events/power_system',
            PowerSystemEvent, self.power_sub)

    def power_sub(self, msg):
        if msg.event == PowerSystemEvent.CHARGE_COMPLETED:
            rospy.loginfo("Charging done")
            self.charged = True
        else:
            rospy.loginfo("Charging")
            self.charged = False

    def execute(self, userdata):
        # block here until we are charged
        while not self.charged and not rospy.is_shutdown():
            rospy.sleep(1)
        # return charged
        return 'charged'

class Undocking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.cmd_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist,
            queue_size=2)
        self.pose = Pose()

    def odom_cb(self, msg):
        self.pose = msg.pose.pose
        
    def execute(self, userdata):
        initial_pose = self.pose
        d = 0
        # back up for 0.5 meters to get away from dock
        while d < 0.5 and not rospy.is_shutdown():
            cmd = Twist()
            cmd.linear.x = -0.2
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.1)
            pose = self.pose
            dx = self.pose.position.x - initial_pose.position.x
            dy = self.pose.position.y - initial_pose.position.y
            d = math.hypot(dx, dy)
        # send a stop command for good measure
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        return 'done'

def random_goal(userdata, goal):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'base_link'
    goal.target_pose.header.stamp = rospy.Time.now()

    # back up one meter
    goal.target_pose.pose.position.x = -1.0
    goal.target_pose.pose.orientation.w = 1.0
    return goal

if __name__ == '__main__':
    try:
        rospy.init_node('random_nav')
        rospy.loginfo('nav demo started')

        sm = smach.StateMachine(['charged', 'done'])

        with sm:
            smach.StateMachine.add('Charging', Charging(),
                transitions={'charged': 'Undocking'})

            smach.StateMachine.add('Undocking', Undocking(),
                           transitions={'done': 'Navigating'})

            smach.StateMachine.add('Navigating',
                            smach_ros.SimpleActionState('move_base',
                                    MoveBaseAction,
                                    goal_cb=random_goal),
                            transitions={'succeeded': 'Navigating',
                                         'aborted': 'Navigating',
                                         'preempted': 'Navigating')

            smach.StateMachine.add('Docking',
                            smach_ros.SimpleActionState('dock_drive_action',
                                    AutoDockingAction),
                            transitions={'succeeded': 'Charging',
                                        'aborted': 'Docking',
                                        'preempted': 'Docking'})

        sm.execute()
            

        #navclient  = NavClient()
        #dockclient = DockClient()


        #dockclient.undock()
        #navclient.send_goal(goal)
        #dockclient.dock()

    except rospy.ROSInterruptException:
        pass
    rospy.loginfo('nav demo done')
