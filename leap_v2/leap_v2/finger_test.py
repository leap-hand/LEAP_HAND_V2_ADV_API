import pybullet as p
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Point, Pose, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from py_utils.rigid_transform import RigidTransform
import sys
import time

class Leapv2Test(Node):
    def __init__(self):
        super().__init__('leapv2_test')    
        self.pub_hand = self.create_publisher(JointState, '/leapv2_node/cmd_raw_leap_r', 10)
        output = np.zeros(17)
        addition = 1
        state = 1
        time.sleep(2)
        while True:
            if state == 1:
                output[1] = output[1] + addition * 0.05
                if output[1] > 1.75:
                    addition = -1
                if output[1] < 0:
                    state = 2
                    addition = 1
            elif state == 2:
                output[2] = output[2] + addition * 0.05
                if output[2] > 1.75:
                    addition = -1
                if output[2] < 0:
                    state = 3
                    addition = 1
            if state == 3:
                output[1] = output[1] + addition * 0.05
                output[2] = output[2] + addition * 0.05
                if output[1] > 1.75:
                    addition = -1
                if output[1] < 0:
                    state = 1
                    addition = 1
            print(output[1])
            time.sleep(0.025)
            stater = JointState()
            stater.position = list(output)
            self.pub_hand.publish(stater)


def main(args=None):
    rclpy.init(args=args)
    leappybulletik = Leapv2Test()
    rclpy.spin(leappybulletik)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    leappybulletik.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()