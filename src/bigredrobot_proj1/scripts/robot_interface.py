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
            self.gripper_at = 0
        elif configuration=='stacked_ascending':
            self.blocks_over = list(range(num_blocks)) # e.g [0, 1, 2]
            self.gripper_at = num_blocks
        elif configuration=='stacked_descending':
            self.blocks_over = list(range(2,num_blocks+1)) # e.g [2, 3, 0]
            self.blocks_over.append(0)
            self.gripper_at = 1
        self.gripper_closed = True       
    
    def init_publisher(self):
        self.pub = rospy.Publisher('state', State, queue_size=10)

    # Callback function for move_robot server
    def handle_move_robot(self, req):
        #TODO: action handling, incorporating conditions for each action and world state 
        success = False
        if req.action==req.ACTION_OPEN_GRIPPER:
            if self.gripper_closed:
                self.gripper_closed = False
                success = True
        elif req.action==req.ACTION_CLOSE_GRIPPER:
            if not self.gripper_closed:
                self.gripper_closed = True
                success = True
        elif req.action==req.ACTION_MOVE_TO:
            if not self.gripper_closed and self.is_topmost(req.target):
                self.gripper_at = req.target
                success = True
        elif req.action==req.ACTION_MOVE_OVER:
            if self.gripper_closed and self.is_topmost(req.target):
                self.blocks_over[self.gripper_at-1] = req.target
                success = True
        rospy.loginfo("Attempt action=%i on block=%i"%(req.action, req.target))
        if success:
            rospy.loginfo("Action successful!")
        return MoveRobotResponse(success)

    def init_service(self):
        self.srv = rospy.Service('move_robot', MoveRobot, self.handle_move_robot) 
    
    def run(self):
        rate = rospy.Rate(1) # 1hz
        while not rospy.is_shutdown():
            state = State()
            state.blocks_over = self.blocks_over
            state.gripper_at = self.gripper_at
            state.gripper_closed = self.gripper_closed
            self.pub.publish(state)
            rate.sleep()

    def is_topmost(self, target):
        if target not in self.blocks_over:
            return True
        else:
            return False


if __name__ == '__main__':
    try:
        robint = RobotInterface()
        robint.init_state()
        robint.init_publisher()
        robint.init_service()
        robint.run()

    except rospy.ROSInterruptException:
        pass
