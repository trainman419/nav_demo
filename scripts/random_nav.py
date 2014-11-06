#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math
import random
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from kobuki_msgs.msg import AutoDockingAction,AutoDockingGoal,PowerSystemEvent
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Twist

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
        self.charged = False
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

class BatteryMonitor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['low_battery', 'battery_ok'])
        self.sub = rospy.Subscriber('/mobile_base/events/power_system',
            PowerSystemEvent, self.power_sub)
        self.low_battery = False

    def power_sub(self, msg):
        if msg.event == PowerSystemEvent.BATTERY_LOW or \
            msg.event == PowerSystemEvent.BATTERY_CRITICAL:
            rospy.loginfo("Low battery!")
            self.low_battery = True
        else:
            rospy.loginfo("Battery OK")
            self.low_battery = False

    def execute(self, userdata):
        if self.low_battery:
            return 'low_battery'
        else:
            return 'battery_ok'

def dict_to_pose(d):
    pose = Pose()
    pose.position.x = d['position']['x']
    pose.position.y = d['position']['y']
    pose.position.z = d['position']['z']
    pose.orientation.x = d['orientation']['x']
    pose.orientation.y = d['orientation']['y']
    pose.orientation.z = d['orientation']['z']
    pose.orientation.w = d['orientation']['w']
    return pose

if __name__ == '__main__':
    try:
        rospy.init_node('random_nav')
        rospy.loginfo('nav demo started')

        dock_pose = rospy.get_param('~dock_pose')
        dock_pose = dict_to_pose(dock_pose)
        dock_goal = MoveBaseGoal()
        dock_goal.target_pose.header.frame_id = 'map'
        dock_goal.target_pose.pose = dock_pose

        nav_poses = rospy.get_param('~poses')
        nav_poses = [dict_to_pose(p) for p in nav_poses]
        rospy.loginfo("Dock pose: %s" % (str(dock_pose)))
        rospy.loginfo("Navigation poses: %s" % (str(nav_poses)))

        def random_goal(userdata, goal):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()

            # pick a random pose!
            goal.target_pose.pose = random.choice(nav_poses)
            rospy.loginfo("Navigating to %s" % (str(goal.target_pose.pose)))
            return goal


        sm = smach.StateMachine(['charged', 'done'])

        with sm:
            smach.StateMachine.add('Charging', Charging(),
                transitions={'charged': 'Undocking'})

            smach.StateMachine.add('Undocking', Undocking(),
                           transitions={'done': 'RandomNav'})

            cc_nav = smach.Concurrence(outcomes=['low_battery','continue'],
                            default_outcome='continue',
                            outcome_map={'low_battery': {'BatteryMonitor':
                                'low_battery'}})

            with cc_nav:
                smach.Concurrence.add('Navigating',
                            smach_ros.SimpleActionState('move_base',
                                    MoveBaseAction,
                                    goal_cb=random_goal))
                smach.Concurrence.add('BatteryMonitor', BatteryMonitor())

            smach.StateMachine.add('RandomNav', cc_nav,
                           transitions={'continue': 'RandomNav',
                                        'low_battery': 'Predock'})

            smach.StateMachine.add('Predock',
                            smach_ros.SimpleActionState('move_base',
                                    MoveBaseAction,
                                    goal=dock_goal),
                            transitions={'succeeded': 'Docking',
                                         'aborted': 'Predock',
                                         'preempted': 'Predock'})

            smach.StateMachine.add('Docking',
                            smach_ros.SimpleActionState('dock_drive_action',
                                    AutoDockingAction),
                            transitions={'succeeded': 'Charging',
                                        'aborted': 'Docking',
                                        'preempted': 'Docking'})

        sm.execute()

    except rospy.ROSInterruptException:
        pass
    rospy.loginfo('nav demo done')
