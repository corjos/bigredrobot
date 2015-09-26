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
        self.num_arms = state.num_arms


    def command_update(self, command):
        self.command = command.data


    def init_subscribers(self):
        rospy.Subscriber("state", State, self.state_update)
        rospy.Subscriber("command", String, self.command_update)


    def is_scattered(self):
        if all([x <= 0 for x in self.blocks_over]):
            return True
        else:
            return False    


    def is_stacked_ascending(self):
        #TODO: rewrite everything
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
            rospy.loginfo("stacking ascending")
            for i in range(1, len(self.blocks_over)+1):
                target = i - 1
                if i == 1:
                    target = -1
                self.move_to_right(i, target)
        elif self.is_scattered():
            for i in range(2, len(self.blocks_over) + 1):
                self.move_to_right(i, i - 1)


    def control_stack_descending(self):
        if self.is_stacked_descending():
            pass

        elif self.is_stacked_ascending():
            rospy.loginfo("stacking descending")
            for i in reversed(range(1, len(self.blocks_over)+1)):
                target = i + 1
                if i == len(self.blocks_over):
                    target = -i
                self.move_to_right(i, target)

        elif self.is_scattered():
            for i in reversed(range(1, len(self.blocks_over))):
                self.move_to_right(i, i + 1)


    def control_scatter(self):
        if self.is_scattered():
            pass

        elif self.is_stacked_descending():
            for i in range(1, len(self.blocks_over) + 1):
                self.move_to_right(i, -i)

        elif self.is_stacked_ascending():
            for i in reversed(range(1, len(self.blocks_over) + 1)):
                self.move_to_right(i, -i)


    def control_odd_even(self):
        #For now, stack in any order, but create two separate odd/even stacks
        top_left = -1
        top_right = -2
        if self.is_stacked_descending():
            for i in range(1, len(self.blocks_over) + 1):
                split_stack(i)

        elif self.is_stacked_ascending():
            for i in reversed(range(1, len(self.blocks_over) + 1)):
                split_stack(i)

        elif self.is_scattered():
            #TODO


    def split_stack(self, currentblock):
        if currentblock % 2 == 1:
            self.move_to_left(currentblock, top_left)
            top_left = currentblock
        else:
            self.move_to_right(currentblock, top_right)
            top_right = currentblock


    def move_to_left(self, blocknum, target):
        self.bimanual_move_to(blocknum, target, -5, -5)

    def move_to_right(self, blocknum, target):
        self.bimanual_move_to(-5, -5, blocknum, target)
        # TODO: check for action failure 
        '''self.move_robot(MoveRobotRequest.ACTION_OPEN_GRIPPER, 0)
        self.move_robot(MoveRobotRequest.ACTION_MOVE_TO, blocknum)
        self.move_robot(MoveRobotRequest.ACTION_CLOSE_GRIPPER, 0)       
        self.move_robot(MoveRobotRequest.ACTION_MOVE_OVER, target)'''
            
    def bimanual_move_to(self, lblocknum, ltarget, rblocknum, rtarget):
        actions = [MoveRobotRequest.ACTION_OPEN_GRIPPER, MoveRobotRequest.ACTION_MOVE_TO,
                    MoveRobotRequest.ACTION_CLOSE_GRIPPER, MoveRobotRequest.ACTION_MOVE_OVER]
        targets = [[0,0], [lblocknum, rblocknum], [0,0], [ltarget, rtarget]]
        for action, target in zip(actions,targets):
            req = MoveRobotRequest()
            for arm in [MoveRobotRequest.LEFT_ARM, MoveRobotRequest.RIGHT_ARM]:
                req.action[arm] = action
                req.target[arm] = target[arm]
            #rospy.loginfo(req)
            self.move_robot(req)

    def control(self):
        if self.command == "scatter":
            self.control_scatter()
        elif self.command == "stack_ascending":
            self.control_stack_ascending()
        elif self.command == "stack_descending":
            self.control_stack_descending()
        else:
            raise ValueError('You suck at typing')
        rospy.loginfo("control completed")
        self.command = None


    def run(self):
        rospy.wait_for_service('move_robot')
        self.move_robot = rospy.ServiceProxy('move_robot', MoveRobot)
        while not rospy.is_shutdown():                
            if self.command :
                self.control()


if __name__ == '__main__':
    try:
        c = Controller()
        c.init_subscribers()
        c.run()
    except rospy.ROSInterruptException:
        pass
