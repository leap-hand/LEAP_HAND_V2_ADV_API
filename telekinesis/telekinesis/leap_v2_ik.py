import pybullet as p
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Point, Pose, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from py_utils.rigid_transform import RigidTransform
import sys

class Leapv2PybulletIK(Node):
    def __init__(self):
        super().__init__('leapv2_pyb_ik')      
        # start pybullet
        #clid = p.connect(p.SHARED_MEMORY)
        clid = p.connect(p.GUI)
        if clid < 0:
            p.connect(p.DIRECT)
        self.is_left = self.declare_parameter('isLeft', False).get_parameter_value().bool_value
        self.protomotions_data = self.declare_parameter('protomotions', False).get_parameter_value().bool_value
        if self.protomotions_data:
            if self.is_left:
                self.protomotions_publisher = self.create_publisher(PoseArray, "/protomotions/leap_v2_left")
            else:
                self.protomotions_publisher = self.create_publisher(PoseArray, "/protomotions/leap_v2_right")
        # load right leap hand      
        # specify 10 links for self.leapEndEffectorIndex in the following sequence
        # thumb, index, middle, ring, pinky
        # tip, dip
        if self.is_left:
            self.pip_dip_pairs = [(9,10), (15,16), (21,22), (27,28)]
            self.leapEndEffectorIndex = [4, 5, 11, 12, 17, 18, 23, 24, 29, 30]
            from ament_index_python.packages import get_package_share_directory
            path_src = get_package_share_directory('nvda_data_collector')
            path_src = path_src + "/../../../../src/py_utils/"
            self.pyb_xyz = [0,0.12,-0.21]
            self.pyb_euler = [1.57, 0, 3.14]
            self.LeapId = p.loadURDF(
                "/home/kshaw/telekinesis_3/src/telekinesis/telekinesis/leap_v2/leap_v2_left/robot.urdf",
                self.pyb_xyz,
                p.getQuaternionFromEuler(self.pyb_euler),
                useFixedBase = True
            )
            self.glove_to_leap_mapping_scale = 1.4
            self.pub_hand = self.create_publisher(JointState, '/leapv2_node/cmd_raw_leap_l', 10)
            self.sub_skeleton = self.create_subscription(PoseArray, "/glove/l_short", self.get_glove_data, 10)
        else:
            self.pip_dip_pairs = [(3,4), (9,10), (15,16), (21,22)]
            self.leapEndEffectorIndex = [29, 30, 5, 6, 11, 12, 17, 18, 23, 24]
            self.LeapId = p.loadURDF(
                "/home/kshaw/telekinesis_3/src/telekinesis/telekinesis/leap_v2/leap_v2_right/robot.urdf",
                [0.005,0.13,-0.17],
                p.getQuaternionFromEuler([1.57, 0, 3.14]),
                useFixedBase = True
            )
            self.glove_to_leap_mapping_scale = 1.4
            self.pub_hand = self.create_publisher(JointState, '/leapv2_node/cmd_raw_leap_r', 10)
            self.sub_skeleton = self.create_subscription(PoseArray, "/glove/r_short", self.get_glove_data, 10)
        
        self.numJoints = p.getNumJoints(self.LeapId)

        p.setGravity(0, 0, 0)
        useRealTimeSimulation = 0
        p.setRealTimeSimulation(useRealTimeSimulation)
        self.create_target_vis()
            
    def create_target_vis(self):
        # load balls
        small_ball_radius = 0.01
        small_ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=small_ball_radius)
        ball_radius = 0.01
        ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
        baseMass = 0.001
        basePosition = [0.25, 0.25, 0]
        
        self.ballMbt = []
        for i in range(0,5):
            self.ballMbt.append(p.createMultiBody(baseMass=baseMass, baseCollisionShapeIndex=ball_shape, basePosition=basePosition)) # for base and finger tip joints    
            no_collision_group = 0
            no_collision_mask = 0
            p.setCollisionFilterGroupMask(self.ballMbt[i], -1, no_collision_group, no_collision_mask)
        p.changeVisualShape(self.ballMbt[0], -1, rgbaColor=[1, 0, 0, 1]) 
        p.changeVisualShape(self.ballMbt[1], -1, rgbaColor=[0, 1, 0, 1]) 
        p.changeVisualShape(self.ballMbt[2], -1, rgbaColor=[0, 0, 1, 1])  
        p.changeVisualShape(self.ballMbt[3], -1, rgbaColor=[1, 1, 0, 1])
        p.changeVisualShape(self.ballMbt[4], -1, rgbaColor=[1, 1, 1, 1])
        
    def update_target_vis(self, hand_pos):
        _, current_orientation = p.getBasePositionAndOrientation( self.ballMbt[0])
        p.resetBasePositionAndOrientation(self.ballMbt[0], hand_pos[3], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[1])
        p.resetBasePositionAndOrientation(self.ballMbt[1], hand_pos[5], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[2])
        p.resetBasePositionAndOrientation(self.ballMbt[2], hand_pos[7], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[3])
        p.resetBasePositionAndOrientation(self.ballMbt[3], hand_pos[9], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[4])
        p.resetBasePositionAndOrientation(self.ballMbt[4], hand_pos[1], current_orientation)
    
    def get_glove_data(self, pose):
        #gets the data converts it and then computes IK and visualizes
        poses = pose.poses
        hand_pos = []  
        for i in range(0,8):
            hand_pos.append([poses[i].position.x * 0.6* self.glove_to_leap_mapping_scale, poses[i].position.y * self.glove_to_leap_mapping_scale * 0.7, -poses[i].position.z * self.glove_to_leap_mapping_scale * 0.93])
        pinky_scale = 1.6
        for i in range(8,10):
            hand_pos.append([poses[i].position.x * 0.6 * pinky_scale, poses[i].position.y * pinky_scale * 0.7, -poses[i].position.z * pinky_scale* 0.95])

        self.compute_IK(hand_pos)
        self.update_target_vis(hand_pos)
        
    def compute_IK(self, hand_pos):
        p.stepSimulation()     
        
        # index, middle, ring, pinky, thumb
        # tip, dip
        
        index_middle_pos = hand_pos[2]
        index_pos = hand_pos[3]
        
        middle_middle_pos = hand_pos[4]
        middle_pos = hand_pos[5]
        
        ring_middle_pos = hand_pos[6]
        ring_pos = hand_pos[7]
        
        pinky_middle_pos = hand_pos[8]
        pinky_pos = hand_pos[9]
        
        thumb_middle_pos = hand_pos[0]
        thumb_pos = hand_pos[1]
        
        leapEndEffectorPos = [
            thumb_pos,
            thumb_middle_pos,
            index_pos,
            index_middle_pos,
            middle_pos,
            middle_middle_pos,
            ring_pos,
            ring_middle_pos,
            pinky_pos,
            pinky_middle_pos,
        ]

        jointPoses = p.calculateInverseKinematics2(
            self.LeapId,
            self.leapEndEffectorIndex,
            leapEndEffectorPos,
            solver=p.IK_DLS,
            maxNumIterations=30,
            residualThreshold=0.001,
        )
        if self.is_left:
            combined_jointPoses = (
                jointPoses[0:4] + (0.0,0.0,) + \
                jointPoses[4:9] + (0.0,0.0,) + \
                jointPoses[9:13] + (0.0,0.0,) + \
                jointPoses[13:17] + (0.0,0.0,) + \
                jointPoses[17:21] + (0.0,0.0,)
            )
        else:
            combined_jointPoses = (
                jointPoses[0:5] + (0.0,0.0,) + \
                jointPoses[5:9] + (0.0,0.0,) + \
                jointPoses[9:13] + (0.0,0.0,) + \
                jointPoses[13:17] + (0.0,0.0,) + \
                jointPoses[17:21] + (0.0,0.0,)
            )
        combined_jointPoses = list(combined_jointPoses)

        
        for pip_id, dip_id in self.pip_dip_pairs:
            avg_joint = (combined_jointPoses[pip_id] + combined_jointPoses[dip_id]) / 2
            combined_jointPoses[pip_id] = avg_joint
            combined_jointPoses[dip_id] = avg_joint
            
        # update the hand joints
        for i in range(31):
            p.setJointMotorControl2(
                bodyIndex=self.LeapId,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=combined_jointPoses[i],
                targetVelocity=0,
                force=500,
                positionGain=0.3,
                velocityGain=1,
            )

         # map results to real robot
        real_robot_hand_q = np.array([0.0 for _ in range(17)])
        combined_jointPoses = np.array(combined_jointPoses)
        if self.is_left:       
            #thumb
            real_robot_hand_q[12:15] = combined_jointPoses[[2,1,3]]
            real_robot_hand_q[12] = -real_robot_hand_q[12]
            real_robot_hand_q[13] = real_robot_hand_q[13]
            #index
            real_robot_hand_q[0:3] = combined_jointPoses[[8,7,9]]
            real_robot_hand_q[1] =real_robot_hand_q[1]
            real_robot_hand_q[0] = -real_robot_hand_q[0]
            #middle 
            real_robot_hand_q[3:6] = combined_jointPoses[[14,13,15]]
            real_robot_hand_q[4] =real_robot_hand_q[4]
            real_robot_hand_q[3] = -real_robot_hand_q[3]
            #ring
            real_robot_hand_q[6:9] = combined_jointPoses[[20,19,21]]
            real_robot_hand_q[7] =real_robot_hand_q[7]
            real_robot_hand_q[6] = -real_robot_hand_q[6]
            #pinky
            real_robot_hand_q[9:12] = combined_jointPoses[[26,25,27]]
            real_robot_hand_q[10] =real_robot_hand_q[10]
            real_robot_hand_q[9] = -real_robot_hand_q[9]
            #two palm joints
            real_robot_hand_q[15] = combined_jointPoses[0]
            real_robot_hand_q[16] = combined_jointPoses[6]
        else:
            #thumb
            real_robot_hand_q[12:15] = combined_jointPoses[[27,26,28]]
            real_robot_hand_q[12] = real_robot_hand_q[12]
            real_robot_hand_q[13] = real_robot_hand_q[13]
            #index
            real_robot_hand_q[0:3] = combined_jointPoses[[2,1,3]]
            real_robot_hand_q[1] =real_robot_hand_q[1]
            real_robot_hand_q[0] = real_robot_hand_q[0]
            #middle 
            real_robot_hand_q[3:6] = combined_jointPoses[[8,7,9]]
            real_robot_hand_q[4] =real_robot_hand_q[4]
            real_robot_hand_q[3] = real_robot_hand_q[3]
            #ring
            real_robot_hand_q[6:9] = combined_jointPoses[[14,13,15]]
            real_robot_hand_q[7] =real_robot_hand_q[7]
            real_robot_hand_q[6] = real_robot_hand_q[6]
            #pinky
            real_robot_hand_q[9:12] = combined_jointPoses[[20,19,21]]
            real_robot_hand_q[10] =real_robot_hand_q[10]
            real_robot_hand_q[9] = real_robot_hand_q[9]
            #two palm joints
            real_robot_hand_q[15] = combined_jointPoses[25]
            real_robot_hand_q[16] = combined_jointPoses[0]
            
        real_robot_hand_q[[1,4,7,10]] = real_robot_hand_q[[1,4,7,10]] + 0.2
        real_robot_hand_q[[2,5,8,11,14]] = real_robot_hand_q[[2,5,8,11,14]]
        
        stater = JointState()
        stater.position = [float(i) for i in real_robot_hand_q]
        self.pub_hand.publish(stater)

        if self.protomotions_data:
            self.publish_rigid_body_xyz()
        
    '''
    This is just for Arthur to get the rigidbody xyz that Protomotions needs
    '''
    def publish_rigid_body_xyz(self):
        if self.is_left:
            x = p.getLinkStates(self.LeapId,[7,8,9,10,13,14,15,16,19,20,21,22,25,26,27,28,13,0,1,2,3])     
            poses_list = []        
            for i in range(0,21):
                #First we convert from Pybullet frame to as if the the wrist was at the zero position
                output =  RigidTransform(from_frame = "zero_world", to_frame = "new_world", xyz = self.pyb_xyz, rpy = self.pyb_euler).inverse() * RigidTransform(from_frame = "link", to_frame = "new_world", xyz = x[i][4], quat = x[i][5])
                #Now we publish.....in integrator we will have to move the wrist around but we won't calculate that here
                poses_list.append(output.to_ros_msg)
            output_array_msg = PoseArray()
            output_array_msg.poses = poses_list
            self.protomotions_publisher.publish(output_array_msg)
        else:
            pass
            



def main(args=None):
    rclpy.init(args=args)

    leapv2pybulletik = Leapv2PybulletIK()

    rclpy.spin(leapv2pybulletik)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    integrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
  