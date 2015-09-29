#!/usr/bin/env python

## Template for robot_interface node

import rospy
from bigredrobot_proj1.srv import *
from bigredrobot_proj1.msg import *

import baxter_interface 

class RobotInterface:
    
    def __init__(self):
        rospy.init_node('robot_interface', anonymous=True)
        self.is_real_robot = not rospy.get_param('symbolic_only')
       if not self.is_sys_sim:
            # initialise baxter
            baxter_interface.RobotEnable().enable()
            self.right_limb = baxter_interface.Limb('right') 
            self.right_gripper = baxter_interface.Gripper('right')
            self.right_gripper.calibrate()
            
            self.TABLE_Z = -0.08
            self.BLOCK_HEIGHT = 0.05


    def init_state(self):
        num_blocks = rospy.get_param('num_blocks')
        configuration = rospy.get_param('configuration')
        self.num_arms = rospy.get_param('num_arms')
        if self.num_arms == 1:
            self.arms = [State.RIGHT_ARM] # If we are only using one arm. We are using right        
        elif self.num_arms == 2:
            self.arms = [State.LEFT_ARM, State.RIGHT_ARM]
        else:
            raise ValueError('Wrong number of arms')
     
        self.gripper_at = [None]*2
        self.gripper_closed = [None]*2
        
        # convention?: initialise right arm at top of stack, left arm over a
        # negative odd-numbered block?
        if configuration=='scattered':
            # Not implemented fully
            self.blocks_over = list(range(-num_blocks+1,1)) # e.g [-2, -1, 0]
            self.gripper_at[State.LEFT_ARM] = -1 # Not technically correct TODO: sort out these conventions
            self.gripper_at[State.RIGHT_ARM] = 0
        elif configuration=='stacked_ascending':
            self.blocks_over = list(range(num_blocks)) # e.g [0, 1, 2]
            self.blocks_over[0] = -1
            self.gripper_at[State.LEFT_ARM] = -1 # Not technically correct TODO: sort out these conventions
            self.gripper_at[State.RIGHT_ARM] = num_blocks
        elif configuration=='stacked_descending':
            self.blocks_over = list(range(2,num_blocks+1)) # e.g [2, 3, 0]
            self.blocks_over.append(-5)
            self.gripper_at[State.LEFT_ARM] = -1 # Not technically correct TODO: sort out these conventions
            self.gripper_at[State.RIGHT_ARM] = 1
            self.gripper_closed[State.LEFT_ARM] = True   
            self.gripper_closed[State.RIGHT_ARM] = True             
        if not self.is_sys_sim:
            # augment world state with coordinates
            pose = self.right_limb.endpoint_pose()
            pos = pose.popitem()
            self.base_x = pos.x
            self.base_y = {i:0 for i in range(-self.num_blocks,0)}
            for i in base_y:
                if i % 2 == 1:
                    self.base_y[i] = pos.y + i*2*self.BLOCK_HEIGHT
                else:
                    self.base_y[i] = pos.y - i*2*self.BLOCK_HEIGHT

            self.block_coords = {i:[0, 0, 0] for i in range(1,self.num_blocks+1)} # [x,y,z]
            
            nextblock = self.gripper_at[State.RIGHT_ARM]
            nextz = pos.z
            while nextblock > 0:
                # TODO: make sure we always start on block 0
                self.block_coords[nextblock] = [self.base_x, self.base_y[0], nextz]
                nextz = nextz - self.BLOCK_HEIGHT
                nextblock = self.blocks_over[nextblock-1]
                
            self.ORIENT = pose.popitem()
            pass



    def init_publisher(self):
        self.pub = rospy.Publisher('state', State, queue_size=10)

    # Callback function for move_robot server
    def handle_move_robot(self, req):    

        # TODO: Cartestian coordinate update    
        for arm in self.arms:
            if req.action[arm]==req.ACTION_OPEN_GRIPPER:
                if self.gripper_closed[arm]:
                    self.robot_open_gripper()
                    self.gripper_closed[arm] = False
                else:
                    rospy.logwarn('Invalid action OPEN_GRIPPER (arm = %i)' %(arm))
                    return False
            elif req.action[arm]==req.ACTION_CLOSE_GRIPPER:
                if not self.gripper_closed[arm]:
                    self.robot_close_gripper()
                    self.gripper_closed[arm] = True
                else:
                    rospy.logwarn('Invalid action CLOSE_GRIPPER(arm = %i)' %(arm))
                    return False
            elif req.action[arm]==req.ACTION_MOVE_TO:
                if not self.gripper_closed[arm] and self.is_topmost(req.target[arm]):
                    self.robot_move_to(req.target[arm])
                    self.gripper_at[arm] = req.target[arm]
                else:
                    rospy.logwarn('Invalid action MOVE_TO (arm = %i, target = %i)' %(arm, req.target[arm]))
                    return False
            elif req.action[arm]==req.ACTION_MOVE_OVER:
                if self.gripper_closed[arm] and self.is_topmost(req.target[arm]):
                    self.robot_move_over(req.target[arm])
                    self.blocks_over[self.gripper_at[arm]-1] = req.target[arm]
                else:
                    rospy.logwarn('Invalid action MOVE_OVER (arm = %i, target = %i, current block = %i)' %(arm, req.target[arm], self.gripper_at[arm]))
                    return False
            elif req.action[arm]==req.ACTION_IDLE:
                pass # nothing to see here

        # TODO: add joint motion constraints/checks
        return False


        
    def robot_open_gripper(self):
        if self.is_real_robot:
            self.right_gripper.open()

    def robot_close_gripper(self):
        if self.is_real_robot:
            self.right_gripper.close()

    def robot_move_to(self, target):
        if self.is_real_robot: 
            x0, y0, z0 = self.block_coords[self.gripper_at[State.RIGHT_ARM]]       
            x1, y1, z1 = self.block_coords[target]  
            self.move_safely(x0,y0,z0,x1,y1,z1)

    def robot_move_over(self, target):
        if self.is_real_robot: 
            x0, y0, z0 = self.block_coords[self.gripper_at[State.RIGHT_ARM]]       
            x1, y1, z1_ = self.block_coords[self.gripper_at[target]]  
            z1 = z1_ + self.BLOCK_HEIGHT            
            self.move_safely(x0,y0,z0,x1,y1,z1)
    
    def move_safely(self,x0,y0,z0,x1,y1,z1)
            move_robot(x0, y0, self.TABLE_z + (self.num_block+2)*self.BLOCK_HEIGHT)
            move_robot(x1, y1, z0 + (self.num_block+2)*self.BLOCK_HEIGHT)
            move_robot(x1, y1, z1)


    def move_robot(self, x, y, z)
            loc = Point(float(x),float(y),float(z))
            hdr = Header(stamp=rospy.Time.now(), frame_id='base')
            ikreq = SolvePositionIKRequest()
            poses = {'right': PoseStamped(header=hdr, pose=Pose(position=loc, orientation=self.ORIENT))}

            ikreq.pose_stamp.append(poses['right'])

            resp = self.ik_solve(ikreq)

            if not resp.isValid[0]:
                rospy.loginfo("Nothin found here...")
            else:
                rospy.loginfo("VALID IK")
            rospy.loginfo(resp)
            
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            rospy.loginfo(limb_joints)
            self.right_limb.move_to_joint_positions(limb_joints)

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

    def baxter_move(source, dest):
        pass
        

if __name__ == '__main__':
    try:
        robint = RobotInterface()
        robint.init_state()
        robint.init_publisher()
        robint.init_service()
        robint.run()

    except rospy.ROSInterruptException:
        pass
