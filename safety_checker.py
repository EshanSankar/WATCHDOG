#TODO: write collision checker and handler for robots moving objecs

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Int8 # -1, 0, 1
import numpy as np
from digital_to_real import Assets

class SafetyChecker(Node):
    def __init__(self):
        super().__init__("safety_checker")
        self.timer = self.create_timer(0.01, self.check_safety)
        self.xarm_sub = self.create_subscription(Float32MultiArray, "/sim_xarm/joint_states", self.xarm_cb, 10)
        self.ot2_sub = self.create_subscription(Float32MultiArray, "/sim_ot2/joint_states", self.ot2_cb, 10)
        self.xarm_joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.xarm_targets = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.ot2_joints = np.array([0.0, 0.0, 0.0])
        self.ot2_targets = np.array([0.0, 0.0, 0.0])

        # 0: continue current action, 1: safe, -1: unsafe
        self.status_pub = self.create_publisher(Int8, "/safety_checker/status_int", 10)
        self.target_asset_sub = self.create_subscription(String, "/safety_checker/target_asset", self.target_asset_cb, 10)
        self.XARM_JOINT_THRESHOLD = 0.05
        self.OT2_JOINT_THRESHOLD = 0.005
        self.OT2_ASSET_THRESHOLD = 0.25
        self.ASSET_ROTATION_THRESHOLD = 0.1
        self.assets = Assets()
        self.asset_subs = {}
        self.asset_poses = {}
        self.initial_asset_poses = {}
        self.target_asset = ""
        self.moving_asset = ""
        for asset in self.assets:
           self.asset_subs[f"{asset}"] = self.create_subscription(Float32MultiArray, f"/sim_{asset}/pose", lambda msg: self.asset_cb(msg, f"{asset}"), 10)
           self.initial_asset_poses[f"{asset}"] = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]) #TODO fix
           self.asset_poses[f"{asset}"] = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    def target_asset_cb(self, msg):
        # there is only going to be 1 target and 1 moving asset at a time
        # msg format: "target_asset, moving_asset"
        raw = msg.data.split(" ")
        self.target_asset, self.moving_asset = raw[0], raw[1]
    def asset_cb(self, msg, asset):
        self.asset_poses[f"{asset}"][0], self.asset_cmd_poses[f"{asset}"][1], self.asset_cmd_poses[f"{asset}"][2] = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        self.asset_poses[f"{asset}"][3], self.asset_cmd_poses[f"{asset}"][4], self.asset_cmd_poses[f"{asset}"][5], self.asset_cmd_poses[f"{asset}"][6] = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
    def xarm_cb(self, msg):
        self.xarm_joints = msg.data[:7]
        self.xarm_targets = msg.data[7:]
    def ot2_cb(self, msg):
        self.ot2_joints = msg.data[:3]
        self.ot2_targets = msg.data[3:]
    def check_collision(self):
        # Check collisions of robots and assets
        return False
    def check_safety(self):
        self.get_logger().info("RUNNING!")
        # Implement safety checks here
        if self.check_collision():
            self.status_pub.publish(Int8(data=-1))
            return
        # Check if xArm is moving
        xarm_moving = False
        for i in range(len(self.xarm_joints)):
            # if xArm is moving
            if abs(self.xarm_joints[i] - self.xarm_targets[i]) > self.XARM_JOINT_THRESHOLD:
                xarm_moving = True
                break
        if xarm_moving:
            self.status_pub.publish(Int8(data=0))
            for asset in self.assets:
                if asset == self.moving_asset:
                    continue
                if (np.linalg.norm(self.asset_poses[f"{asset}"][:3] - self.ot2_targets[:2]) > self.OT2_ASSET_THRESHOLD
                    or np.linalg.norm(self.asset_poses[f"{asset}"][3:] - self.initial_asset_poses[f"{asset}"][3:]) > self.ASSET_ROTATION_THRESHOLD):
                    self.status_pub.publish(Int8(data=-1))
                    return
                # if gripper is closing, check if the asset is not blocked
                if (self.xarm_joints[7] - self.xarm_targets[7] > self.XARM_JOINT_THRESHOLD):
                    if asset == self.target_asset:
                        continue
                    # if gripper is coming from above, check if asset is blocked from above
                    if xarm_joint is above target asset: #TODO check gripper position
                        if ((np.linalg.norm(self.asset_poses[f"{asset}"][:2] - self.asset_poses[f"{self.target_asset}"][:2]) < self.OT2_ASSET_THRESHOLD)
                            and (self.asset_poses[f"{asset}"][2] > self.asset_poses[f"{self.target_asset}"][2])):
                            self.status_pub.publish(Int8(data=-1))
                            return
                    # if gripper is coming from left, check if asset is blocked from left
                    if xarm_joint is left of target asset:
                        if ((np.linalg.norm(self.asset_poses[f"{asset}"][0:3:2] - self.asset_poses[f"{self.target_asset}"][0:2:2]) < self.OT2_ASSET_THRESHOLD)
                            and (self.asset_poses[f"{asset}"][0] < self.asset_poses[f"{self.target_asset}"][0])):
                            self.status_pub.publish(Int8(data=-1))
                            return
                    # if gripper is coming from right, check if asset is blocked from right
                    if xarm_joint is right of target asset:
                        if ((np.linalg.norm(self.asset_poses[f"{asset}"][1:3] - self.asset_poses[f"{self.target_asset}"][1:3]) < self.OT2_ASSET_THRESHOLD)
                            and (self.asset_poses[f"{asset}"][0] > self.asset_poses[f"{self.target_asset}"][0])):
                            self.status_pub.publish(Int8(data=-1))
                            return
            return
                        
        # Check if OT2 is moving
        ot2_moving = False
        for i in range(len(self.ot2_joints)):
            if abs(self.ot2_joints[i] - self.ot2_targets[i] > self.OT2_JOINT_THRESHOLD):
                ot2_moving = True
                break
        if ot2_moving:
            self.status_pub.publish(Int8(data=0))
            for asset in self.assets:
                if asset == self.moving_asset:
                    continue
                # check if assets are in the correct positions
                if (np.linalg.norm(self.asset_poses[f"{asset}"][:3] - self.ot2_targets[:2]) > self.OT2_ASSET_THRESHOLD
                    or np.linalg.norm(self.asset_poses[f"{asset}"][3:] - self.initial_asset_poses[f"{asset}"][3:]) > self.ASSET_ROTATION_THRESHOLD):
                    self.status_pub.publish(Int8(data=-1))
                    return
                # if pipette is lowering, check if the below asset is not blocked
                if (self.ot2_joints[2] - self.ot2_targets[2] > self.OT2_JOINT_THRESHOLD):
                    if asset == self.target_asset:
                        continue
                    if ((np.linalg.norm(self.asset_poses[f"{asset}"][:2] - self.asset_poses[f"{self.target_asset}"][:2]) < self.OT2_ASSET_THRESHOLD)
                        and (self.asset_poses[f"{asset}"][2] > self.asset_poses[f"{self.target_asset}"][2])):
                        self.status_pub.publish(Int8(data=-1))
                        return
            return

        # If robots are stationary and everything is fine, publish 1
        self.status_pub.publish(Int8(data=1))

def main():
    rclpy.init()
    safety_checker = SafetyChecker()
    rclpy.spin(safety_checker)
    safety_checker.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()