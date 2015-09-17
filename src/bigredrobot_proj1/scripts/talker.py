#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from bigredrobot_proj1.srv import *

# Callback function for move_robot server
def handle_move_robot(req):
    print "Attempt action=%i on block=%i"%(req.action, req.target)
    result = True
    print "Action successful!"
    return MoveRobotResponse(result)

# Main function that creates node
def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    s = rospy.Service('move_robot', MoveRobot, handle_move_robot) 
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
