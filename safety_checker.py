# TO BE RUN SEPARATELY FROM ISAAC SIM; THIS FILE IS JUST HERE FOR NOW

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Int8 # -1, 0, 1
import numpy as np

class SafetyChecker(Node):
    def __init__(self):
        super().__init__("safety_checker")
        self.xarm_sub = self.create_subscription(Float32MultiArray, "/sim_xarm/joint_states", self.xarm_cb, 10)
        self.ot2_sub = self.create_subscription(Float32MultiArray, "/sim_ot2/joint_states", self.ot2_cb, 10)
        self.xarm_joints = np.array(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.xarm_targets = np.array(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.ot2_joints = np.array(0.0, 0.0, 0.0, 0.0, 0.0)
        self.ot2_targets = np.array(0.0, 0.0, 0.0, 0.0, 0.0)
        self.status_pub = self.create_publisher(Int8, "safety_checker/status", 10)
        self.status = Int8(data=1) # 0: continue current action, 1: safe, -1: unsafe
        self.XARM_JOINT_THRESHOLD = 0.05
        self.OT2_JOINT_THRESHOLD = 0.05
    def xarm_cb(self, msg):
        self.xarm_joints = msg.data[:7]
        self.xarm_targets = msg.data[7:]
    def ot2_cb(self, msg):
        self.ot2_joints = msg.data[:5]
        self.ot2_targets = msg.data[5:]
    def check_safety(self):
        rclpy.spin_once(self, timeout_sec=0.0)
        self.status.data = 1
        # Implement safety checks here
        if False: #if safety is false
            self.status.data = -1
        for i in range(len(self.xarm_joints)):
            if abs(self.xarm_joints[i] - self.xarm_targets[i]) > self.XARM_JOINT_THRESHOLD:
                self.status.data = 0
        for i in range(len(self.ot2_joints)):
            if abs(self.ot2_joints[i] - self.ot2_targets[i]) > self.OT2_JOINT_THRESHOLD:
                self.status.data = 0
        self.status_pub.publish(self.status)

def main():
    rclpy.init()
    safety_checker = SafetyChecker()
    safety_checker.check_safety()
    safety_checker.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()