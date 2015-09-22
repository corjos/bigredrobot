#!/usr/bin/env python

## Template for controller node

import rospy
from bigredrobot_proj1.msg import *
from bigredrobot_proj1.srv import *

def callback(data):
    print data.states

def controller():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node("controller", anonymous=True)

    rospy.Subscriber("state", State, callback)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

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
