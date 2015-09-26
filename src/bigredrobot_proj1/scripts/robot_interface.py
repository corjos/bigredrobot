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
        self.num_arms = rospy.get_param('num_arms')

        self.gripper_at = [None]*2
        self.gripper_closed = [None]*2
        
        # convention?: initialise right arm at top of stack, left arm over a
        # negative odd-numbered block?
        if configuration=='scattered':
            # Not implemented fully
            self.blocks_over = list(range(-num_blocks+1,1)) # e.g [-2, -1, 0]
            self.gripper_at[State.LEFT_ARM] = -1
            self.gripper_at[State.RIGHT_ARM] = 0
        elif configuration=='stacked_ascending':
            self.blocks_over = list(range(num_blocks)) # e.g [0, 1, 2]
            self.gripper_at[State.LEFT_ARM] = -1
            self.gripper_at[State.RIGHT_ARM] = num_blocks
        elif configuration=='stacked_descending':
            self.blocks_over = list(range(2,num_blocks+1)) # e.g [2, 3, 0]
            self.blocks_over.append(0)
            self.gripper_at[State.LEFT_ARM] = -1
            self.gripper_at[State.RIGHT_ARM] = 1
            self.gripper_closed[State.LEFT_ARM] = True   
            self.gripper_closed[State.RIGHT_ARM] = True     
    
    def init_publisher(self):
        self.pub = rospy.Publisher('state', State, queue_size=10)

    # Callback function for move_robot server
    def handle_move_robot(self, req):
        if self.num_arms == 1:
            arms = [State.RIGHT_ARM] # If we are only using one arm. We are using right        
        elif self.num_arms == 2:
            arms = [State.LEFT_ARM, State.RIGHT_ARM]
        else:
            raise ValueError('Wrong number of arms')
        
        for arm in arms:
            if req.action[arm]==req.ACTION_OPEN_GRIPPER:
                if self.gripper_closed[arm]:
                    self.gripper_closed[arm] = False
                else:
                    raise ValueError('Invalid action OPEN_GRIPPER (arm = %i)' %(arm))
            elif req.action[arm]==req.ACTION_CLOSE_GRIPPER:
                if not self.gripper_closed[arm]:
                    self.gripper_closed[arm] = True
                else:
                    raise ValueError('Invalid action CLOSE_GRIPPER(arm = %i)' %(arm))
            elif req.action[arm]==req.ACTION_MOVE_TO:
                if not self.gripper_closed[arm]:
                    if req.target[arm] > 0 and self.is_topmost(req.target[arm]): # Block is real                    
                        self.gripper_at[arm] = req.target[arm]
                    else: # Explicitly handle trying to move to phantom block by leaving state unchanged
                        # TODO: Think of a better way to deal with this
                        rospy.logwarn('Ignored invalid MOVE_TO action and left state unchanged')
                else:
                    raise ValueError('Invalid action MOVE_TO (arm = %i, target = %i)' %(arm, req.target[arm]))
            elif req.action[arm]==req.ACTION_MOVE_OVER:
                if self.gripper_closed[arm] and self.is_topmost(req.target[arm]):
                    self.blocks_over[self.gripper_at[arm]-1] = req.target[arm]
                else:
                    raise ValueError('Invalid action MOVE_OVER (arm = %i, target = %i)' %(arm, req.target[arm]))
            elif req.action[arm]==req.ACTION_IDLE:
                pass # nothing to see here
        success = True # More useful on the real robot? Controller should never request symbolically invalid actions, so a ValueError will be used for that case.
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
            state.num_arms = self.num_arms
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
