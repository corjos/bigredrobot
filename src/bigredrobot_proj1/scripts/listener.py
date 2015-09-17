#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from bigredrobot_proj1.srv import *

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

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
    listener()
