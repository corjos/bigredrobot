#!/usr/bin/env python

## Template for robot_interface node

import rospy
from bigredrobot_proj1.srv import *
from bigredrobot_proj1.msg import *

# Callback function for move_robot server
def handle_move_robot(req):
    rospy.loginfo("Attempt action=%i on block=%i"%(req.action, req.target))
    result = True
    rospy.loginfo("Action successful!")
    return MoveRobotResponse(result)

# Main function that creates node
def robot_interface():
    
    # Query world state params
    num_blocks = rospy.get_param('num_blocks')
    configuration = rospy.get_param('configuration')
    if configuration=='scattered':
        states = list(range(-num_blocks+1,1)) # e.g [-2, -1, 0]
    elif configuration=='stacked_ascending':
        states = list(range(num_blocks)) # e.g [0, 1, 2]
    elif configuration=='stacked_descending':
        states = list(reversed(range(num_blocks))) # e.g [2, 1, 0]
    pub = rospy.Publisher('state', State, queue_size=10)
    s = rospy.Service('move_robot', MoveRobot, handle_move_robot) 
    rospy.init_node('robot_interface', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(states)
        rate.sleep()

if __name__ == '__main__':
    try:
        robot_interface()
    except rospy.ROSInterruptException:
        pass
