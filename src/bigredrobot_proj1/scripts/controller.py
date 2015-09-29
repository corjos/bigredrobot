#!/usr/bin/env python

## Template for controller node

import rospy
import baxter_interface
import logging
LOG_FILENAME = "/home/cs4752/Documents/bigredrobot/debug.log"


from std_msgs.msg import String
from bigredrobot_proj1.msg import *
from bigredrobot_proj1.srv import *

#Should probably go to robot_interface
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

class Controller():
    
    def __init__(self):
        logging.debug("Testing")
        logging.debug("Another line")
        rospy.init_node("controller", anonymous=True)
        self.command = None
        #Baxter enable
        baxter_interface.RobotEnable().enable()
        #Baxter Init right arm
        self.right_limb = baxter_interface.Limb('right')

        self.right_gripper = baxter_interface.Gripper('right')
        self.right_gripper.calibrate()
        

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


    def make_available(self, target):
        rospy.logwarn('Make %i available, States: %s' %(target, str(self.blocks_over)))
        #Do the following if the target isn't available yet
        if not self.is_available(target):
            blockOnTop = self.blocks_over.index(target) + 1 #Correct for zero index
            rospy.loginfo('%i is on top of %i' %(blockOnTop,target))
            if not self.is_available(blockOnTop):
                self.make_available(blockOnTop)
            rospy.loginfo('Moving %i to %i' %(blockOnTop, -blockOnTop))
            self.move_left(blockOnTop, -blockOnTop)
        else:
            rospy.loginfo('Block %i is available, States: %s' %(target, str(self.blocks_over)))
        

    def is_available(self, target):
        return target not in self.blocks_over


    def control_stack_ascending(self):
        self.make_available(1)
        self.make_available(2)
        self.make_available(3)
       
        #for i in range(1, len(self.blocks_over) + 1):
            #self.make_available(i)
            #destination = -i if i == 1 else i - 1
            #self.move(i, destination)


    def control_stack_descending(self):
        for i in reversed(range(1, len(self.blocks_over) + 1)):
            self.make_available(i)
            destination = -i if i == 1 else i - 1
            self.move(i, destination)


    def control_scatter(self):
        for i in range(len(self.blocks_over)):
            self.make_available(self.blocks_over[i])
            destination = -self.blocks_over[i]
            self.move(self.blocks_over[i], destination)


    def control_stack_odd_even(self):
        #For now, stack in any order, but create two separate odd/even stacks
        top_left = -1 # Top block in left stack
        top_right = -2
        if self.is_stacked_descending():
            for i in range(1, len(self.blocks_over) + 1):
                top_left, top_right = self.split_stack(i, top_left, top_right)

        elif self.is_stacked_ascending():
            for i in reversed(range(1, len(self.blocks_over) + 1)):
                top_left, top_right = self.split_stack(i, top_left, top_right)

        elif self.is_scattered():
            pass

    def split_stack(self, currentblock, top_left, top_right):
        # move a given block on top of block top_left if it is odd, or top right if it is even
        # return the topmost block in the left and right stacks
        if currentblock % 2 == 1:
            self.move_left(currentblock, top_left)
            top_left = currentblock
        else:
            self.move_right(currentblock, top_right)
            top_right = currentblock
        return top_left, top_right

    
    #I don't know if we need this anymore...
    def get_base_block(self, blocknum):
        currentBlock = blocknum
        while currentBlock > 0:
            currentBlock = self.blocks_over[currentBlock - 1]
        return currentBlock


    def move(self, blocknum, target):
        if blocknum % 2 == 1:
            self.move_left(blocknum, target)
        else:
            self.move_right(blocknum, target)

    def move_left(self, blocknum, target):
        self.bimanual_move(blocknum, target, None, None)

    def move_right(self, blocknum, target):
        self.bimanual_move(None, None, blocknum, target)
        # TODO: check for action failure 
            
    def bimanual_move(self, lblocknum, ltarget, rblocknum, rtarget):
        # Execute open->move_to->close->move_over sequence in both arms, simultaneously moving blocks lblocknum and rblocknum on top of ltarget and rtarget, respectively. When block and target values are both None the respective arm will be idle.
        actions = [MoveRobotRequest.ACTION_OPEN_GRIPPER, MoveRobotRequest.ACTION_MOVE_TO,
                    MoveRobotRequest.ACTION_CLOSE_GRIPPER, MoveRobotRequest.ACTION_MOVE_OVER]
        targets = [[0,0], [lblocknum, rblocknum], [0,0], [ltarget, rtarget]]
        for action, target in zip(actions,targets):
            req = MoveRobotRequest()
            for arm in [MoveRobotRequest.LEFT_ARM, MoveRobotRequest.RIGHT_ARM]:               
                if target[arm] is None:
                    req.action[arm] = MoveRobotRequest.ACTION_IDLE
                    req.target[arm] = 0
                else:                
                    req.action[arm] = action
                    req.target[arm] = target[arm]
            #rospy.loginfo(req)
            self.move_robot(req)

    def move_arm(self, command_in):
        logging.debug("I think I'll move")
        #x - 2
        #y - 3
        #z - 4
        coordinates = command_in.split('_')
        rospy.loginfo(coordinates)

        '''angles = self.right_limb.joint_angles()
        self.right_limb.move_to_joint_positions(angles)
        self.right_limb.move_to_neutral()
        neutral_angles = self.right_limb.joint_angles()
        logging.debug(neutral_angles)
        self.right_limb.move_to_joint_positions(angles)'''


        #Development code --- not pretty at all sorry
        ikreq = SolvePositionIKRequest()
        pose = self.right_limb.endpoint_pose()
        b = True
        pos = pose.popitem()
        orient = pose.popitem()
        prev = pos[1]

        loc = Point(float(coordinates[2]),float(coordinates[3]),float(coordinates[4]))

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        '''poses = {
            'right': PoseStamped(header=hdr,
                pose=Pose(position=loc, orientation=orient[1]))}'''

        poses = { #z -.08 is the table BLOCKS are roughly .05
                'right': PoseStamped(header=hdr,
                    pose=Pose(position=loc, orientation=Quaternion(
                            x=0.74,
                            y=0.67,
                            z=0.027,
                            w=0.025)
                    )
                )
        }
                
        ikreq.pose_stamp.append(poses['right'])

        #def ik_solve(limb, pos, orient):
        resp = self.ik_solve(ikreq)
        
        if not resp.isValid[0]:
            rospy.loginfo("Nothin found here...")
        else:
            rospy.loginfo("VALID IK")
        rospy.loginfo(resp)
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        rospy.loginfo(limb_joints)
       
        self.right_limb.move_to_joint_positions(limb_joints)

    def move_neutral(self):
        self.right_limb.move_to_neutral()
        pose = self.right_limb.endpoint_pose()
        rospy.loginfo(pose)

    def open_gripper(self):
        self.right_gripper.open()
    
    def close_gripper(self):
        self.right_gripper.close()

    def get_pose(self):
        rospy.loginfo(self.right_limb.endpoint_pose())
        


    def control(self):
        logging.debug("I got a command")
        if self.command == "scatter":
            self.control_scatter()
        elif self.command == "stack_ascending":
            self.control_stack_ascending()
        elif self.command == "stack_descending":
            self.control_stack_descending()
        elif self.command == "stack_odd_even":
            if self.num_arms == 2:
                self.control_stack_odd_even()
            else:
                rospy.logwarn('odd_even only available in bimanual mode, try again')
        elif "move_arm" in self.command:
            logging.debug("I should move my arm")
            self.move_arm(self.command)
        elif self.command == "move_neutral":
            self.move_neutral()
        elif self.command == "open_gripper":
                    self.open_gripper()
        elif self.command == "close_gripper":
                    self.close_gripper()
        elif self.command == "pose":
                    self.get_pose()
        else:
            rospy.logwarn('You suck at typing. invalid name, try again.')
        self.command = None


    def run(self):
        self.pub = rospy.Publisher('debug_out', String, queue_size=10)
        rospy.wait_for_service('move_robot')
        self.move_robot = rospy.ServiceProxy('move_robot', MoveRobot)

        #IK Service
        self.ik_solve = rospy.ServiceProxy("ExternalTools/right/PositionKinematicsNode/IKService", SolvePositionIK)
        
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
