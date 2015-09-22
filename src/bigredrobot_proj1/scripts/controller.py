#!/usr/bin/env python

## Template for controller node

import rospy
from std_msgs.msg import String
from bigredrobot_proj1.msg import *
from bigredrobot_proj1.srv import *

def state_update(state):
    #TODO: allow passing of state between functions. Perhaps use a class with callbacks as member functions which modify instance variables.
    print state.blocks_over
    print state.gripper_over
    print state.gripper_closed

def command_update(command):
    #TODO: allow passing of state between functions. Perhaps use a class with callbacks as member functions which modify instance variables.
    print command

def controller():

    rospy.init_node("controller", anonymous=True)

    rospy.Subscriber("state", State, state_update)
    rospy.Subscriber("command", String, command_update)

    rospy.wait_for_service('move_robot')
    rate = rospy.Rate(.5)
    while not rospy.is_shutdown():
        try:
            move_robot = rospy.ServiceProxy('move_robot', MoveRobot)
            action = 1
            target = 2
            result = move_robot(action, target)
            print result
            rate.sleep()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


if __name__ == '__main__':
    controller()
