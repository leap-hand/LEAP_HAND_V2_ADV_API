
import pybullet as p
import numpy as np

def compute_IK(hand_pos, LeapId, leapEndEffectorIndex):
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
        index_pos,
        index_middle_pos,
        middle_pos,
        middle_middle_pos,
        ring_pos,
        ring_middle_pos,
        pinky_pos,
        pinky_middle_pos,
        thumb_pos,
        thumb_middle_pos,
    ]

    jointPoses = p.calculateInverseKinematics2(
        LeapId,
        leapEndEffectorIndex,
        leapEndEffectorPos,
        solver=p.IK_DLS,
        maxNumIterations=50,
        residualThreshold=0.0001,
    )
    for i, pose in enumerate(jointPoses):
        print(i, pose)
    

    # combined_jointPoses = (jointPoses[0:4] + (0.0,) + jointPoses[4:8] + (0.0,) + jointPoses[8:12] + (0.0,) + jointPoses[12:16] + (0.0,))
    # combined_jointPoses = list(combined_jointPoses)

    # # update the hand joints
    # for i in range(20):
    #     p.setJointMotorControl2(
    #         bodyIndex=LeapId,
    #         jointIndex=i,
    #         controlMode=p.POSITION_CONTROL,
    #         targetPosition=combined_jointPoses[i],
    #         targetVelocity=0,
    #         force=500,
    #         positionGain=0.3,
    #         velocityGain=1,
    #     )


    # # map results to real robot
    # real_robot_hand_q = np.array([0.0 for _ in range(16)])
    # #real_left_robot_hand_q = np.array([0.0 for _ in range(16)])

    # real_robot_hand_q[0:4] = jointPoses[0:4]
    # real_robot_hand_q[4:8] = jointPoses[4:8]
    # real_robot_hand_q[8:12] = jointPoses[8:12]
    # real_robot_hand_q[12:16] = jointPoses[12:16]
    # real_robot_hand_q[0:2] = real_robot_hand_q[0:2][::-1]
    # real_robot_hand_q[4:6] = real_robot_hand_q[4:6][::-1]
    # real_robot_hand_q[8:10] = real_robot_hand_q[8:10][::-1]
    # stater = JointState()
    # stater.position = np.array(real_robot_hand_q)
    # self.pub_hand.publish(stater)
        
if __name__=="__main__":
    # start pybullet
    clid = p.connect(p.SHARED_MEMORY)
    if clid < 0:
        p.connect(p.GUI)

    
    LeapId = p.loadURDF(
        "/home/jimyoung/telekinesis_2/src/leap_v2/leap_v2_right/robot.urdf",
        [0,0,0],
        p.getQuaternionFromEuler([0,0,0]),
        useFixedBase = True
    )
    
    base_info = p.getBodyInfo(LeapId)
    base_name = base_info[1].decode('utf-8')
    print(f"Base name: {base_name}")

    # Get the number of joints
    num_joints = p.getNumJoints(LeapId)
    print(f"Number of joints: {num_joints}")

    # Loop through all joints to get their names and indices
    for joint_index in range(num_joints):
        joint_info = p.getJointInfo(LeapId, joint_index)
        joint_name = joint_info[1].decode('utf-8')
        link_name = joint_info[12].decode('utf-8')
        print(f"Joint index: {joint_index}, Joint name: {joint_name}, Link name: {link_name}")


    leapEndEffectorIndex = [4, 5, 11, 12, 17, 18, 23, 24, 29, 30]
    hand_pos = np.zeros((10, 3))
    
    compute_IK(hand_pos, LeapId, leapEndEffectorIndex)
    num_joints = p.getNumJoints(LeapId)
    print(f"Number of joints: {num_joints}")

    # Dictionary to store link index and link name
    link_index_name_dict = {}

    # Loop through all joints to get their link indices and link names
    for joint_index in range(num_joints):
        joint_info = p.getJointInfo(LeapId, joint_index)
        link_index = joint_info[0]
        link_name = joint_info[12].decode('utf-8')
        link_index_name_dict[link_index] = link_name
        print(f"Link index: {link_index}, Link name: {link_name}")

    # Print the complete dictionary
    print("Link index and name dictionary:")
    print(link_index_name_dict)
    
    