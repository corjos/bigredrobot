#!/usr/bin/env python

## Template for robot_interface node

import rospy
from bigredrobot_proj1.srv import *
from bigredrobot_proj1.msg import *

# Callback function for move_robot server
def handle_move_robot(req):
    #TODO: action handling, incorporating conditions for each action and world state     (pass as global?)
    if req.action==req.ACTION_OPEN_GRIPPER:
        pass
    elif req.action==req.ACTION_CLOSE_GRIPPER:
        pass
    elif req.action==req.ACTION_MOVE_TO:
        pass
    elif req.action==req.ACTION_MOVE_OVER:
        pass

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
        # Not implemented fully
        blocks_over = list(range(-num_blocks+1,1)) # e.g [-2, -1, 0]
        gripper_over = 0
    elif configuration=='stacked_ascending':
        blocks_over = list(range(num_blocks)) # e.g [0, 1, 2]
        gripper_over = blocks_over[num_blocks-1]
    elif configuration=='stacked_descending':
        blocks_over = list(range(2,num_blocks+1)) # e.g [2, 3, 0]
        blocks_over.append(0)
        gripper_over = blocks_over[0]
    gripper_closed = True

    pub = rospy.Publisher('state', State, queue_size=10)
    s = rospy.Service('move_robot', MoveRobot, handle_move_robot) 
    rospy.init_node('robot_interface', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        pub.publish(State(blocks_over, gripper_over, gripper_closed))
        rate.sleep()

if __name__ == '__main__':
    try:
        robot_interface()
    except rospy.ROSInterruptException:
        pass
