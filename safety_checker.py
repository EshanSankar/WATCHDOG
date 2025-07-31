# TO BE RUN SEPARATELY FROM ISAAC SIM; THIS FILE IS JUST HERE FOR NOW

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import numpy as np

class SafetyChecker(Node):
    def __init__(self):
        super().__init__("safety_checker")
        self.joint_sub = self.create_subscription(Float32MultiArray, "/sim_world/poses", self.joint_cb, 10)
    def joint_cb(self, msg):
        xarm_joints = msg.data[:7]
        ot2_joints = msg.data[7:]
        # DO LOGIC HERE
        return

def main():
    rclpy.init()
    safety_checker = SafetyChecker()
    rclpy.spin(safety_checker)
    safety_checker.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()