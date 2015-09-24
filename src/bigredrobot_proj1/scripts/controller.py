#!/usr/bin/env python

## Template for controller node

import rospy
from std_msgs.msg import String
from bigredrobot_proj1.msg import *
from bigredrobot_proj1.srv import *

class Controller():
    
    def __init__(self):
        rospy.init_node("controller", anonymous=True)
        self.command = None

    def state_update(self, state):
        self.blocks_over = state.blocks_over
        self.gripper_at = state.gripper_at
        self.gripper_closed = state.gripper_closed

    def command_update(self, command):
        self.command = command

    def init_subscribers(self):
        rospy.Subscriber("state", State, self.state_update)
        rospy.Subscriber("command", String, self.command_update)

    def is_stacked_ascending(self):
        # Can't we just check the current state? wasn't this passed in as a configuration
        if self.blocks_over[0] > 0:
            return False
        for i in range(1,len(self.blocks_over)):
            if self.blocks_over[i] != i:
                return False
        return True
        
    def is_stacked_descending(self):
        #TODO: rewrite everything
        if self.blocks_over[-1] > 0:
            return False
        for i in range(len(self.blocks_over)-1):
            if self.blocks_over[i] != i + 2:
                return False
        return True

    def control_stack_ascending(self):
        if self.is_stacked_ascending():
            pass
        elif self.is_stacked_descending():
            for i in range(1, len(self.blocks_over)+1):
                # TODO: check for action failure 
                # TODO: Reference MoveRobot.ACTION_OPEN_GRIPPER instead of hardcoded 0
                # Didn't like MoveRobot.COMMAND.... not sure why
                self.move_robot(MoveRobotRequest.ACTION_OPEN_GRIPPER, 0) 
                self.move_robot(MoveRobotRequest.ACTION_MOVE_TO, i)
                self.move_robot(MoveRobotRequest.ACTION_CLOSE_GRIPPER, 0) 
                if i == 1:
                    self.move_robot(MoveRobotRequest.ACTION_MOVE_OVER, -1) #MoveRobot.ACTION_MOVE_OVER
                else:
                    self.move_robot(MoveRobotRequest.ACTION_MOVE_OVER, i-1)

    def control_stack_descending(self):
        if self.is_stacked_descending():
            pass
        elif self.is_stacked_ascending():
            for i in reversed(range(1, len(self.blocks_over)))
                self.move_robot(0, 0)
                self.move_robot(2, i)
                self.move_robot(1, 0)
                if i == len(self.blocks_over)
                    self.move_robot(3, -i)
                else:
                    self.move_robot(3, i+1)

            
    def control(self):
        #rospy.loginfo(self.command)
        if self.command == "scatter":
            pass
        elif "stack_ascending" in str(self.command):
            self.control_stack_ascending()
        elif self.command == "stack_descending":
            pass

    def run(self):
        rospy.wait_for_service('move_robot')
        self.move_robot = rospy.ServiceProxy('move_robot', MoveRobot)
        while not self.command:
            pass
        self.control()


if __name__ == '__main__':
    c = Controller()
    c.init_subscribers()
    c.run()
