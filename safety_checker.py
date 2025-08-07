#TODO: for checking blockage, check each of xyz instead of np.norm

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Int8 # -1, 0, 1
from sensor_msgs.msg import JointState
import numpy as np
import json

ASSET_PATH = "file:///home/sdl1/isaacsim/sdl1-digital-twin/assets"
ORCHESTRATOR_PATH = "" #"/home/sdl1/ros2_projects/digital_twin/src/catalyst-OT2-xArm"
OT2_BASE_HEIGHT = 0.054
OT2_SLOT_DIMS = {"x": 0.04, "y": 0.025}
OT2_COORDS = {
	1: [0.0, 0.0], 2: [0.13, 0.0], 3: [0.26, 0.0],
	4: [0.0, 0.09], 5: [0.13, 0.09], 6: [0.26, 0.09],
	7: [0.0, 0.18], 8: [0.13, 0.18], 9: [0.26, 0.18],
	10: [0.0, 0.27], 11: [0.13, 0.27], 12: [0.26, 0.27]}

def LoadAssets(asset_path=ASSET_PATH):
	assets = {}
	with open(f"{ORCHESTRATOR_PATH}xarm_workflow.json", "r") as f:
		workflow = json.load(f)
		for asset_type, asset in workflow["global_config"]["labware"].items():
			assets[asset_type] = [asset["slot"], np.array([
				OT2_COORDS[asset["slot"]][0] + OT2_SLOT_DIMS["x"],
				OT2_COORDS[asset["slot"]][1] + OT2_SLOT_DIMS["y"],
				OT2_BASE_HEIGHT])]
	return assets

class SafetyChecker(Node):
    def __init__(self):
        super().__init__("safety_checker")
        self.timer = self.create_timer(0.01, self.check_safety)
        self.xarm_sub = self.create_subscription(Float32MultiArray, "/sim_xarm/joint_states", self.xarm_cb, 10)
        self.xarm_gripper_sub = self.create_subscription(JointState, "/sim_xarm/gripper_position", self.xarm_gripper_cb, 10)
        self.ot2_sub = self.create_subscription(Float32MultiArray, "/sim_ot2/joint_states", self.ot2_cb, 10)
        self.xarm_joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.xarm_targets = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.xarm_gripper_position = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        self.ot2_joints = np.array([0.0, 0.0, 0.0])
        self.ot2_targets = np.array([0.0, 0.0, 0.0])

        # 0: continue current action, 1: safe, -1: unsafe
        self.status_pub = self.create_publisher(Int8, "/safety_checker/status_int", 10)
        self.target_asset_ot2_sub = self.create_subscription(String, "/sim_ot2/target_asset", self.target_asset_ot2_cb, 10)
        self.target_asset_xarm_sub = self.create_subscription(String, "/sim_xarm/target_asset", self.target_asset_xarm_cb, 10)
        self.XARM_JOINT_THRESHOLD = 0.05
        self.XARM_GRIPPER_THRESHOLD = 0.1
        self.OT2_JOINT_THRESHOLD = 0.005
        self.OT2_ASSET_THRESHOLD = 0.25 # account for asset width, etc.
        self.ASSET_ROTATION_THRESHOLD = 0.1
        self.assets = LoadAssets()
        self.asset_subs = {}
        self.asset_poses = {}
        self.initial_asset_poses = {}
        self.target_asset_xarm = ""
        self.target_asset_ot2 = ""
        for asset_type, _ in self.assets.items():
           self.asset_subs[asset_type] = self.create_subscription(Float32MultiArray, f"/sim_{asset_type}/pose", lambda msg, asset_type=asset_type: self.asset_cb(msg, asset_type), 10)
           self.initial_asset_poses[asset_type] = np.copy(self.assets[asset_type][1])
           self.asset_poses[asset_type] = np.copy(self.assets[asset_type][1])
    def target_asset_ot2_cb(self, msg):
        self.target_asset_ot2 = msg.data
    def target_asset_xarm_cb(self, msg):
        self.target_asset_xarm = msg.data
    def asset_cb(self, msg, asset_type):
        self.asset_poses[asset_type][0], self.asset_cmd_poses[asset_type][1], self.asset_cmd_poses[asset_type][2] = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        self.asset_poses[asset_type][3], self.asset_cmd_poses[asset_type][4], self.asset_cmd_poses[asset_type][5], self.asset_cmd_poses[asset_type][6] = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
    def xarm_cb(self, msg):
        self.xarm_joints = msg.data[:8]
        self.xarm_targets = msg.data[8:]
    def xarm_gripper_cb(self, msg):
        self.xarm_gripper_position = msg.data
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
        for i in range(len(self.xarm_joints) - 2): # gripper doesn't count as moving
            # if xArm is moving
            if abs(self.xarm_joints[i] - self.xarm_targets[i]) > self.XARM_JOINT_THRESHOLD:
                xarm_moving = True
                break
        if xarm_moving:
            self.status_pub.publish(Int8(data=0))
            for asset_type, _ in self.assets.items():
                if asset == self.target_asset_xarm: # if the object is being carried, ignore it
                    continue
                if (np.linalg.norm(self.asset_poses[asset_type][:3] - self.ot2_targets[:2]) > self.OT2_ASSET_THRESHOLD) or \
                    (np.linalg.norm(self.asset_poses[asset_type][3:] - self.initial_asset_poses[asset_type][3:]) > self.ASSET_ROTATION_THRESHOLD):
                    self.status_pub.publish(Int8(data=-1))
                    return
        # Check if assets are available for gripping/releasing
        if self.target_asset_xarm != "":
            if self.xarm_targets[7] < self.xarm_joints[7]: # check if gripper is closing; can use [7] or [8]
                # if gripper is coming from above, check if asset is blocked from above
                if self.xarm_gripper_position[2] > self.asset_poses[self.target_asset_xarm][2] + self.XARM_GRIPPER_THRESHOLD:
                    for asset_type, _ in self.assets.items():
                        if asset_type == self.target_asset_xarm:
                            continue
                        if ((np.linalg.norm(self.asset_poses[asset_type][:2] - self.asset_poses[self.target_asset_xarm][:2]) < self.OT2_ASSET_THRESHOLD) and
                            (self.asset_poses[asset_type][2] > self.asset_poses[self.target_asset_xarm][2])):
                            self.status_pub.publish(Int8(data=-1))
                            return
                # if gripper is coming from left, check if asset is blocked from left or from above
                if self.xarm_gripper_position[0] > self.asset_poses[self.target_asset_xarm][0] + self.XARM_GRIPPER_THRESHOLD:
                    for asset_type, _ in self.assets.items():
                        if asset_type == self.target_asset_xarm:
                            continue
                        if (np.linalg.norm(self.asset_poses[asset_type][:2] - self.asset_poses[self.target_asset_xarm][:2]) < self.OT2_ASSET_THRESHOLD) or \
                            ((np.linalg.norm(self.asset_poses[asset_type][:2] - self.asset_poses[self.target_asset_xarm][:2]) < self.OT2_ASSET_THRESHOLD) and
                             (self.asset_poses[asset_type][2] > self.asset_poses[self.target_asset_xarm][2])):
                            self.status_pub.publish(Int8(data=-1))
                            return
                # if gripper is coming from right, check if asset is blocked from right or from above
                if self.xarm_gripper_position[0] < self.asset_poses[self.target_asset_xarm][0] - self.XARM_GRIPPER_THRESHOLD:
                    for asset_type, _ in self.assets.items():
                        if asset_type == self.target_asset_xarm:
                            continue
                        if (np.linalg.norm(self.asset_poses[asset_type][:2] - self.asset_poses[self.target_asset_xarm][:2]) < self.OT2_ASSET_THRESHOLD) or \
                            ((np.linalg.norm(self.asset_poses[asset_type][:2] - self.asset_poses[self.target_asset_xarm][:2]) < self.OT2_ASSET_THRESHOLD) and
                             (self.asset_poses[asset_type][2] > self.asset_poses[self.target_asset_xarm][2])):
                            self.status_pub.publish(Int8(data=-1))
                            return
            elif self.xarm_targets[7] > self.xarm_joints[8]: # gripper is releasing
                return #TODO

            return
                        
        # Check if OT2 is moving
        ot2_moving = False
        for i in range(len(self.ot2_joints)):
            if abs(self.ot2_joints[i] - self.ot2_targets[i] > self.OT2_JOINT_THRESHOLD):
                ot2_moving = True
                break
        if ot2_moving:
            self.status_pub.publish(Int8(data=0))
            for asset_type, _ in self.assets.items():
                if asset_type == self.target_asset_xarm: # if the object is being carried, ignore it
                    continue
                # check if assets are in the correct positions
                if (np.linalg.norm(self.asset_poses[asset_type][:3] - self.ot2_targets[:2]) > self.OT2_ASSET_THRESHOLD) or \
                    (np.linalg.norm(self.asset_poses[asset_type][3:] - self.initial_asset_poses[asset_type][3:]) > self.ASSET_ROTATION_THRESHOLD):
                    self.status_pub.publish(Int8(data=-1))
                    return
                # if pipette is lowering, check if the below asset is not blocked
                if (self.ot2_joints[2] - self.ot2_targets[2] > self.OT2_JOINT_THRESHOLD):
                    if asset == self.target_asset_ot2:
                        continue
                    if ((np.linalg.norm(self.asset_poses[asset_type][:2] - self.asset_poses[self.target_asset_ot2][:2]) < self.OT2_ASSET_THRESHOLD) and
                        (self.asset_poses[asset][2] > self.asset_poses[self.target_asset_ot2][2])):
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