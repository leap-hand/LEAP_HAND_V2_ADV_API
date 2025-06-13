import rclpy
from rclpy.node import Node
import sys
import numpy as np

import math
import time
from typing import Optional, Union
import csv
import copy

from sensor_msgs.msg import JointState
from std_msgs.msg import String
#from alfred_interfaces.srv import LeapPosition, LeapVelocity, LeapEffort

import leap_v2_utils.dynamixel_client as dxl
import leap_v2_utils.leap_v2_utils as lv2u

#######################################################

class LeapvtwoNode(Node):
    def __init__(
        self,
        device = None):
        super().__init__('leapv2_node')
        port = self.declare_parameter('port',"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT78LTIG-if00-port0").get_parameter_value().string_value
        self.get_logger().info(f"{port}")
        isLeft = self.declare_parameter('isLeft',False).get_parameter_value().bool_value
        self.curr_pos = np.zeros(17)
        
        from ament_index_python.packages import get_package_share_directory
        path_src = get_package_share_directory("leap_v2")
        path_src = path_src + "/../../../../src/leap_v2/aligner/alignments"
        if isLeft:
            path_src = path_src + "/test_L.csv"
        else:
            path_src = path_src + "/test_R.csv"
        self.disable_palm = False
        #self.map_type = rospy.get_param('/leaphand_node/map_type', 'allegro')
        # self.ema_amount = float(rospy.get_param('/leaphand_node/ema', '1.0')) #take only current
        #LOG.info("outside loop")
        #########INIT HAND
        self.motors_side =    [0,3,6,9,12]
        self.motors_forward = [1,4,7,10,13]
        self.motors_curl =    [2,5,8,11,14]
        self.motors_palm =    [15,16]  # 15 is for the thumb, 16 is between the 4 fingers, 
        self.all_motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]

        #self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 57600)
        self.dxl_client = dxl.DynamixelClient(self.all_motors, port, 4000000)
        self.dxl_client.connect()
        
        self.strength_scale = 1.0
        
        #Configure MCP Side Motors (XC330)   
        self.dxl_client.sync_write(self.motors_side, np.ones(len(self.motors_side))*5, 11, 1)  #set into Current-based Position Control Mode
        self.dxl_client.set_torque_enabled(self.motors_side, True)
        self.dxl_client.sync_write(self.motors_side, np.ones(len(self.motors_side)) * 250 * self.strength_scale, 102, 2)  #Current limit 500ma 

        #Configure MCP forward Motors (XM430)
        self.open_align, self.close_align = self._read_limits(path_src)  ##First Read alignment file to know where zeros are.
        self.limits_min_max = np.sort(np.stack([self.open_align, self.close_align]), axis=0) #make min and max sorted smallest to largest so you can use in np.clip later
        
        self.dxl_client.sync_write(self.motors_forward, np.ones(len(self.motors_forward))*5, 11, 1)   #Set into extended-current position control mode because we don't have current-based mode on XC430
        self.dxl_client.sync_write(self.motors_forward, np.ones(len(self.motors_forward)) * (650 * self.strength_scale/2.69), 102, 2)  #Current limit 700 * 2.69   
        self.dxl_client.set_torque_enabled(self.motors_forward, True)
        
        #Configure Curl Motors (XM430)     
        self.dxl_client.sync_write(self.motors_curl, np.ones(len(self.motors_curl))*5, 11, 1)  #set into Current-based Position Control Mode
        self.dxl_client.sync_write(self.motors_curl, np.ones(len(self.motors_curl)) * (700 * self.strength_scale/2.69), 102, 2)  #Current limit 800 * 2.69
        self.dxl_client.set_torque_enabled(self.motors_side, True)
        
        #Configure Palm Motors (XM430)
        self.dxl_client.sync_write(self.motors_palm, np.ones(len(self.motors_palm))*5, 11, 1)  #set into Current-based Position Control Mode
        self.dxl_client.sync_write(self.motors_palm, np.ones(len(self.motors_palm)) * (700 * self.strength_scale /2.69), 102, 2)  #Current limit 800 * 2.69
        self.dxl_client.set_torque_enabled(self.motors_palm, True)
        
        curr_pos = self.dxl_client.read_pos()  
        #print(self.limits_min_max)
        #if we are less than 10 degrees less than the actual min or 10 degrees more than the actual max then we are probably flipped 360 around on the motor home pose. 
        #The motor always initializes between 0 and 360 degrees.  Compensate for that in software.
        all_in_range = False
        self.home_pose_offset = np.zeros(17)
        #print(curr_pos)
        #print(self.limits_min_max)
        for i in range(0,17):
            if (curr_pos[i] +  self.home_pose_offset[i]) < (self.limits_min_max[0][i] - 2):
                self.home_pose_offset[i] = self.home_pose_offset[i]  + 6.28
                all_in_range = False
                #print(curr_pos[i])
                #print(self.limits_min_max[0][i])
                #print(i)
            if (curr_pos[i] + self.home_pose_offset[i]) > (self.limits_min_max[1][i] + 2):
                self.home_pose_offset[i] = self.home_pose_offset[i] - 6.28
                all_in_range = False 
                #print(curr_pos[i])
                #print(self.limits_min_max[1][i])
                #print(i)
        #print(self.home_pose_offset)
            
        ##Start all motors
        self.dxl_client.set_torque_enabled(self.all_motors, True)
        #Start subscribers and services
        if isLeft:
            self.create_subscription(JointState, "cmd_leap_l", self._receive_pose, 10)
            self.create_subscription(JointState, "/leapv2_node/cmd_raw_leap_l",  self._receive_raw_joints, 10)
            self.create_subscription(JointState, "cmd_ones_l", self._receive_ones, 10)
            #self.create_service(LeapPosition, 'leap_position', self.pos_srv)
            #self.create_service(LeapVelocity, 'leap_velocity',  self.vel_srv)
            #self.create_service(LeapEffort,'leap_effort',  self.eff_srv)
        else:
            self.create_subscription(JointState, "cmd_leap_r", self._receive_pose, 10)
            self.create_subscription(JointState, "/leapv2_node/cmd_raw_leap_r",  self._receive_raw_joints, 10)
            self.create_subscription(JointState, "cmd_ones_r", self._receive_ones, 10)
            #self.create_service(LeapPosition, 'leap_position', self.pos_srv)
            #self.create_service(LeapVelocity, 'leap_velocity',  self.vel_srv)
            #self.create_service(LeapEffort,'leap_effort',  self.eff_srv)
    '''
    This receives a 20 DOF pose array from the glove and parses it to command the hand.  These are the conventions:
    Finger Order: index[0:4], middle[4:8], ring[8:12], pinky[12:16], thumb[16:20]
    Joint order: MCP side[0,4,8,12,16], MCP forward[1,5,9,13,(17 actually palm thumb forward)], PIP[2,6,10,14,(18 actually MCP thumb forward)], DIP[3,7,11,15,(19 actually thumb curl)]
    Thumb is MCP side, Palm Motor forward, MCP(axle here) forward, PIP (one joint in the finger)

    For MCP side, its -1.57 to 1.57 with 0 in the middle
    For MCP forward, its 0 for fully open and positive up to fully closed ~3 radians.  The top palm joint take the min of the 4 MCP forwards and uses that command up to about 1 radian.  It subtracts that from the forward joint commands.
    For PIP and DIP, its 0 for fully uncurled and 1.57 for fully curled.  We average them first because we only can command one finger joint.  Then we map the 0-1.57 onto the angles that are possible on the motor linearly.
    '''
    
    '''
    This commands a 17 dimensional vector (curr_pos) to the 17 motors.
    Finger Order: index[0:3], middle[3:6], ring[6:9], pinky[9:12], thumb[12:15], palm_thumb[15], palm_fingers[16]
    Joint order: MCP side[0,3,6,9,12], MCP forward[1,4,7,10,13], Curl[2,5,8,11,14], palm_thumb[15], palm_fingers[16]
    '''
    def _receive_pose(self, pose):
        pose = pose.position
        self.prev_pos = copy.deepcopy(self.curr_pos)
        pose = np.array(pose)
        self.curr_input = np.array(lv2u.compress(pose[0:4]) + lv2u.compress(pose[4:8]) + lv2u.compress(pose[8:12]) + lv2u.compress(pose[12:16]) + pose[16:20].tolist() + [0])
        #print(self.curr_input)
        #for MCP side, set 0 to be straight forward by adding 180 to map to robot space  (including motor 12 for thumb)
        zero_to_one = lv2u.unscale(self.curr_input[self.motors_side], -1.57, 1.57)
        self.curr_pos[self.motors_side] = lv2u.scale(zero_to_one, self.open_align[self.motors_side], self.close_align[self.motors_side])

        #for MCP forward, set the fully open pose to 0 and then positive to close the hand.  Will have to check orientation of hand to make sure positive and negatives are correct.
        #The palm motor will move the most so that the minimum MCP forward joint is flat out.
        if not self.disable_palm:
            min_forward = np.min(list(self.curr_input[self.motors_forward[0:4]]) + [0.8])  #(60 degrees or less to set on the palm joint)
            self.curr_input[self.motors_forward[0:4]] = self.curr_input[self.motors_forward[0:4]] - min_forward
            #Set the palm motor
            zero_to_one = lv2u.unscale(min_forward, 0, 1.22)  
            zero_to_one = np.clip(zero_to_one, 0.005, 0.995)
            self.curr_pos[16] = lv2u.scale(zero_to_one, self.open_align[16], self.close_align[16])
        #set the mcp forward motors
        zero_to_one = lv2u.unscale(self.curr_input[self.motors_forward],np.zeros(len(self.motors_forward)),[1.57, 1.57, 1.57, 1.57, 1.57]) #[2.28, 2.059, 2.059, 2.28,2.28]
        zero_to_one = np.clip(zero_to_one, 0.005, 0.995)
        self.curr_pos[self.motors_forward] = lv2u.scale(zero_to_one, self.open_align[self.motors_forward], self.close_align[self.motors_forward])
        #for curl, make it so that you control motor from 0 to 1.  Then we assume the soft printed finger can move 1.57 rad at each joint.  We then map angle input to this script to 0,1
        zero_to_one = lv2u.unscale(self.curr_input[self.motors_curl], 0, 1.57) 
        zero_to_one = np.clip(zero_to_one, 0.005, 0.995)
        self.curr_pos[self.motors_curl] = lv2u.scale(zero_to_one, self.open_align[self.motors_curl], self.close_align[self.motors_curl])
        ##thumb (mcp) forward
        zero_to_one = lv2u.unscale(self.curr_input[14], 0, 2.28)
        zero_to_one = np.clip(zero_to_one, 0.005, 0.995)
        self.curr_pos[13] = lv2u.scale(zero_to_one, self.open_align[13], self.close_align[13])
        ##curl
        zero_to_one = lv2u.unscale(self.curr_input[15], 0, 1.57)
        zero_to_one = np.clip(zero_to_one, 0.005, 0.995)
        self.curr_pos[14] = lv2u.scale(zero_to_one, self.open_align[14], self.close_align[14]) #not a typo on 14/15 on prev
        ##palm next
        zero_to_one = lv2u.unscale(self.curr_input[13], 0, 1.04)  
        zero_to_one = np.clip(zero_to_one, 0.005, 0.995)
        self.curr_pos[15] = lv2u.scale(zero_to_one, self.open_align[15], self.close_align[15]) #not a typo on 14/13 on prev
        #print(self.curr_pos)
        
        self.dxl_client.write_desired_pos(self.all_motors, self.curr_pos)


    '''
    This receives 17 dof motor joints from 0 -> closed angle.
    Finger Order: index = [0:3], middle = [3:6], ring = [6:9], pinky = [9:12], thumb = [12:15], palm_thumb = [15], palm_fingers = [16]
    Joint order: MCP side[0,3,6,9,12], MCP forward[1,4,7,10,(13 actually thumb mcp forward)], PIP/DIP [2,5,8,11,(14 actually MCP thumb tendon)] palm_thumb = [15], palm_4_fingers = [16]
    '''
    def _receive_raw_joints(self, pose):
        # self.get_logger().info("Received raw joints")
        pose = pose.position
        self.prev_pos = copy.deepcopy(self.curr_pos)
        self.curr_input = np.array(pose)
        
        #for MCP side, set 0 to be straight forward by adding 180 to map to robot space  (including motor 12 for thumb)
        zero_to_one = lv2u.unscale(self.curr_input[self.motors_side], -1.57, 1.57)
        self.curr_pos[self.motors_side] = lv2u.scale(zero_to_one, self.open_align[self.motors_side], self.close_align[self.motors_side])
        
        #set the mcp forward motors
        zero_to_one = lv2u.unscale(self.curr_input[self.motors_forward],np.zeros(len(self.motors_forward)),[1.57, 1.57, 1.57, 1.57, 1.57]) #[2.28, 2.059, 2.059, 2.28,2.28]
        zero_to_one = np.clip(zero_to_one, 0.005, 0.995)
        self.curr_pos[self.motors_forward] = lv2u.scale(zero_to_one, self.open_align[self.motors_forward], self.close_align[self.motors_forward])
        
        #for curl, make it so that you control motor from 0 to 1.  Then we assume the soft printed finger can move 1.57 rad at each joint.  We then map angle input to this script to 0,1
        zero_to_one = lv2u.unscale(self.curr_input[self.motors_curl], 0, 1.57) 
        zero_to_one = np.clip(zero_to_one, 0.005, 0.995)
        self.curr_pos[self.motors_curl] = lv2u.scale(zero_to_one, self.open_align[self.motors_curl], self.close_align[self.motors_curl])
        
        ##thumb palm forward
        zero_to_one = lv2u.unscale(self.curr_input[15], 0, 1.04)  
        zero_to_one = np.clip(zero_to_one, 0.005, 0.995)
        self.curr_pos[15] = lv2u.scale(zero_to_one, self.open_align[15], self.close_align[15])    
        ##4 fingers palm forward
        zero_to_one = lv2u.unscale(self.curr_input[16], 0, 1.22)  
        zero_to_one = np.clip(zero_to_one, 0.005, 0.995)
        self.curr_pos[16] = lv2u.scale(zero_to_one, self.open_align[16], self.close_align[16])
        
        #print(self.curr_pos - self.home_pose_offset)
        self.dxl_client.write_desired_pos(self.all_motors, self.curr_pos - self.home_pose_offset)

    def _receive_ones(self, pose):
        raise NotImplementedError
    
    #Checks if we are moving a lot or stuck still periodically.  If we still should be moving set pwm limits high to move fast. If we should be done moving we set the pwm limit lower to not overload the forward XC430 motors. 
    # def _forward_check(self):
    #     self.timesteps_to_move = self.timesteps_to_move - 1
    #     motors_to_remove_pwm_now = []
    #     for i in range(0,len(self.motors_forward)):
    #         if self.timestep_to_move[i]== 0 or self.timestep_to_move[i]== -5:
    #             motors_to_remove_pwm_now[i].append(self.motors_forward[i])
    #         if self.timesteps_to_move[i] < -1000000:
    #             self.timesteps_to_move[i] = -500
    #     if len(motors_to_remove_pwm_now) > 0:
    #         self.dxl_client.sync_write(motors_to_remove_pwm_now, np.ones(len(motors_to_remove_pwm_now)) * 350, 100, 2)
        
    def pos_srv(self, request, response):
        x = self.dxl_client.read_pos()
        x = x + self.home_pose_offset
        x = x.tolist()
        response.position = x
        return response #returns pose in radians
    def vel_srv(self, request, response):
        response.velocity = self.dxl_client.read_vel().tolist()
        return response
    def eff_srv(self, request, response):
        response.effort = self.dxl_client.read_cur().tolist()
        return response
    def _read_limits(self, file_name):
        file_path = file_name
        lower = [] #home position
        upper = [] #closed position
        with open(file_path, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',', quotechar='|')
            for row in reader:
                lower.append(math.radians(float(row[0])))
                upper.append(math.radians(float(row[1])))
        return np.array(lower), np.array(upper)
    def shutdownhook(self):
        self.new_pos = lv2u.scale([0.66, 0.66, 0.66, 0.66, 0.66], self.open_align[self.motors_curl], self.close_align[self.motors_curl])
        time.sleep(1)
        self.new_pos = lv2u.scale([0.2, 0.2, 0.2, 0.2, 0.2], self.open_align[self.motors_curl], self.close_align[self.motors_curl])
        self.dxl_client.write_desired_pos(self.motors_curl, self.new_pos - self.home_pose_offset[self.motors_curl])
        time.sleep(1)
        self.dxl_client.set_torque_enabled(self.motors_curl, False)
        time.sleep(0.0001)
        self.dxl_client.sync_write(self.motors_curl, np.ones(len(self.motors_curl))*0, 11, 1)
        self.dxl_client.sync_write(self.motors_curl, np.ones(12)*[0,], 102, 2) 
        self.dxl_client.set_torque_enabled(self.motors_curl, True)
        print("")
        print("WAIT, LEAP V2 SHUTTING DOWN!!!")
        time.sleep(2)
        for i in range(0,10):
            self.dxl_client.set_torque_enabled(self.all_motors, False)
            time.sleep(0.05)
#init the arm node
def main(args=None):
    rclpy.init(args=args)
    leapv2_node = LeapvtwoNode()
    rclpy.spin(leapv2_node)
    leapv2_node.destroy_node()
    rclpy.shutdown()
'''
Init node
'''
if __name__ == "__main__":
    main()
