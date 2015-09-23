#!/usr/bin/env python

## Template for robot_interface node

import rospy
from bigredrobot_proj1.srv import *
from bigredrobot_proj1.msg import *

class RobotInterface:
    
    def __init__(self):
        rospy.init_node('robot_interface', anonymous=True)
               
    def init_state(self):
        num_blocks = rospy.get_param('num_blocks')
        configuration = rospy.get_param('configuration')
        if configuration=='scattered':
            # Not implemented fully
            self.blocks_over = list(range(-num_blocks+1,1)) # e.g [-2, -1, 0]
            self.gripper_over = 0
        elif configuration=='stacked_ascending':
            self.blocks_over = list(range(num_blocks)) # e.g [0, 1, 2]
            self.gripper_over = self.blocks_over[num_blocks-1]
        elif configuration=='stacked_descending':
            self.blocks_over = list(range(2,num_blocks+1)) # e.g [2, 3, 0]
            self.blocks_over.append(0)
            self.gripper_over = self.blocks_over[0]
        self.gripper_closed = True       
    
    def init_publisher(self):
        self.pub = rospy.Publisher('state', State, queue_size=10)

    # Callback function for move_robot server
    def handle_move_robot(self, req):
        #TODO: action handling, incorporating conditions for each action and world state 
        if req.action==req.ACTION_OPEN_GRIPPER:
            pass
        elif req.action==req.ACTION_CLOSE_GRIPPER:
            pass
        elif req.action==req.ACTION_MOVE_TO:
            pass
        elif req.action==req.ACTION_MOVE_OVER:
            pass
        rospy.loginfo("Attempt action=%i on block=%i"%(req.action, req.target))
        result = True # Action successful?
        rospy.loginfo("Action successful!")
        return MoveRobotResponse(result)

    def init_service(self):
        self.srv = rospy.Service('move_robot', MoveRobot, self.handle_move_robot) 
    
    def run(self):
        rate = rospy.Rate(1) # 1hz
        while not rospy.is_shutdown():
            state = State()
            state.blocks_over = self.blocks_over
            state.gripper_over = self.gripper_over
            state.gripper_closed = self.gripper_closed
            self.pub.publish(state)
            rate.sleep()



if __name__ == '__main__':
    try:
        robint = RobotInterface()
        robint.init_state()
        robint.init_publisher()
        robint.init_service()
        robint.run()

    except rospy.ROSInterruptException:
        pass
