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
        output[3] = 1
        output[6] = 1
        output[9] = 1
        output[1] = 0.1
        output[4] = 0.1
        output[7] = 0.1
        output[10] = 0.1
        addition = 1
        state = 1
        time.sleep(2)
        while True:
            if state == 1:
                output[0] = output[0] + addition * 0.05
                if output[0] > 0.4:
                    addition = -1
                if output[0] < -1.0:
                    state = 2
            elif state == 2:
                output[0] = output[0] + 0.05
                if output[0] > 0:
                    state = 3
            elif state == 3:
                output[1] = output[1] + 0.05
                if output[1] > 1.35:
                    state = 4
                    addition = 1
            elif state == 4:
                output[0] = output[0] + addition * 0.05
                if output[0] > 1.0:
                    addition = -1
                if output[0] < -1.0:
                    state = 5
            elif state == 5:
                output[0] = output[0] + 0.05
                if output[0] > 0:
                    state = 6
            elif state == 6:
                output[1] = output[1] - 0.05
                if output[1] < 0:
                    state = 1
                    addition = 1
            print(state)
            print(output[0], output[1])
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