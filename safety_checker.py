import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Int8
from sensor_msgs.msg import JointState
import numpy as np
import json

ASSET_PATH = "file:///home/sdl1/isaacsim/sdl1-digital-twin/assets"
WORKFLOW_PATH = "video_workflow.json"
OT2_BASE_HEIGHT = 0.054
OT2_OFFSETS = {"x": -0.12756, "y": -0.17065}
OT2_COORDS = {
	1: [0.0, 0.0], 2: [0.13, 0.0], 3: [0.26, 0.0],
	4: [0.0, 0.09], 5: [0.13, 0.09], 6: [0.26, 0.09],
	7: [0.0, 0.18], 8: [0.13, 0.18], 9: [0.26, 0.18],
	10: [0.0, 0.27], 11: [0.13, 0.27], 12: [0.26, 0.27]}
OTHER_ASSETS = {"vial_1": [7, -0.046, 0.022, 0.037], "vial_2": [7, 0.046, -0.022, 0.037], "vial_rack_lid": [7, -0.0635, -0.0425, 0.065]}

def LoadAssets(asset_path=ASSET_PATH):
	assets = {}
	with open(f"{WORKFLOW_PATH}", "r") as f:
		workflow = json.load(f)
		for asset_type, asset in workflow["global_config"]["labware"].items():
			assets[asset_type] = [asset["slot"], np.array([
                    OT2_COORDS[asset["slot"]][0] + OT2_OFFSETS["x"],
                    OT2_COORDS[asset["slot"]][1] + OT2_OFFSETS["y"],
                    OT2_BASE_HEIGHT,
                    1, 0, 0, 0])]
			if asset_type == "tip_rack" or asset_type == "reactor":
				assets[asset_type][1][3:] = np.array([0.7071068, 0, 0, -0.7071068])
			# Because of non-standard assets, some assets will have their origins at their (0,0) instead of the center of the asset
			# Since the above calculation is done w/ r.t. the center of the asset, will need to offset
			if asset_type == "vial_rack":
				assets[asset_type][1][:3] = np.array([
				OT2_COORDS[asset["slot"]][0] + OT2_OFFSETS["x"] - 0.0635,
				OT2_COORDS[asset["slot"]][1] + OT2_OFFSETS["y"] - 0.0425,
				OT2_BASE_HEIGHT])
	for asset_type, offset in OTHER_ASSETS.items():
		assets[asset_type] = [offset[0], np.array([
               OT2_COORDS[offset[0]][0] + OT2_OFFSETS["x"] + offset[1],
               OT2_COORDS[offset[0]][1] + OT2_OFFSETS["y"] + offset[2],
               OT2_BASE_HEIGHT + offset[3],
               1, 0, 0, 0])]
	return assets

class SafetyChecker(Node):
    def __init__(self):
        super().__init__("safety_checker")
        self.timer = self.create_timer(0.01, self.check_safety)
        self.xarm_sub = self.create_subscription(Float32MultiArray, "/sim_xarm/joint_states", self.xarm_cb, 10)
        self.xarm_gripper_sub = self.create_subscription(JointState, "/sim_xarm/gripper_position", self.xarm_gripper_cb, 10)
        self.ot2_pipette_sub = self.create_subscription(JointState, "/sim_ot2/pipette_position", self.ot2_pipette_cb, 10)
        self.ot2_sub = self.create_subscription(Float32MultiArray, "/sim_ot2/joint_states", self.ot2_cb, 10)
        self.xarm_joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.xarm_targets = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.xarm_gripper_position = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        self.ot2_joints = np.array([0.0, 0.0, 0.0])
        self.ot2_targets = np.array([0.0, 0.0, -0.025])
        self.ot2_pipette_position = np.array([0.0, 0.0, 0.0])
        # STATUS INTS: 0: continue current action, 1: safe, -1: unsafe
        self.status_int = 1
        self.status_pub = self.create_publisher(Int8, "/safety_checker/status_int", 10)
        self.xarm_control = 0
        self.xarm_control_sub = self.create_subscription(Int8, "/orchestrator/resolution", self.xarm_control_cb, 10)
        self.target_asset_ot2_sub = self.create_subscription(String, "/sim_ot2/target_asset", self.target_asset_ot2_cb, 10)
        self.target_asset_xarm_sub = self.create_subscription(String, "/sim_xarm/target_asset", self.target_asset_xarm_cb, 10)
        self.XARM_JOINT_THRESHOLD = 0.05 # check if xArm is moving
        self.XARM_GRIPPER_THRESHOLD = 0.1 # xArm grasping position
        self.XARM_CARRY_THRESHOLD = 0.1 # xArm carrying
        self.OT2_JOINT_THRESHOLD = 0.5 # check if OT2 is moving
        self.OT2_ASSET_POSITION_THRESHOLD = 0.5 # check assets are in correct spots relative to pipette
        self.ASSET_ROTATION_THRESHOLD = 999 # check if assets are oriented properly
        self.assets = LoadAssets()
        self.asset_subs = {}
        self.asset_poses = {}
        self.initial_asset_poses = {}
        self.target_asset_xarm = ""
        self.target_asset_ot2 = ""
        self.xarm_carrying = False
        for asset_type, _ in self.assets.items():
           self.asset_subs[asset_type] = self.create_subscription(Float32MultiArray, f"/sim_{asset_type}/pose",
                                                                  lambda msg, asset_type=asset_type: self.asset_cb(msg, asset_type), 10)
           self.initial_asset_poses[asset_type] = np.copy(self.assets[asset_type][1])
           self.asset_poses[asset_type] = np.copy(self.assets[asset_type][1])
    
    # Callback functions
    def target_asset_ot2_cb(self, msg):
        self.target_asset_ot2 = msg.data
    def target_asset_xarm_cb(self, msg):
        self.target_asset_xarm = msg.data
    def asset_cb(self, msg, asset_type):
        self.asset_poses[asset_type] = msg.data
    def xarm_cb(self, msg):
        self.xarm_joints = msg.data[:8]
        self.xarm_targets = msg.data[8:]
    def xarm_gripper_cb(self, msg):
        self.xarm_gripper_position = msg.data
    def ot2_cb(self, msg):
        self.ot2_joints = msg.data[:3]
        self.ot2_targets = msg.data[3:]
    def ot2_pipette_cb(self, msg):
        self.ot2_pipette_position = msg.data
    def xarm_control_cb(self, msg):
        self.xarm_control = 1 if msg.data == 1 else 0
    # Primary functions
    def xarm_collision(self):
        # Check if xArm collides with the OT2, table surface, etc.
        return False
    def check_safety(self):
        self.get_logger().info("RUNNING!")

        # 0. If the state was unsafe but xArm was teleopped to resolve it
        if self.status_int == -1:
            if self.xarm_control == 1:
                self.status_pub.publish(Int8(data=-1))
                return
            self.xarm_control = 0
            self.status_int = 1
        self.get_logger().info("Checked xArm control")
        # 1. Check if xArm arm is moving (gripper doesn't count as moving)
        xarm_moving = False
        for i in range(len(self.xarm_joints) - 2): # Exclude gripper joints
            if abs(self.xarm_joints[i] - self.xarm_targets[i]) > self.XARM_JOINT_THRESHOLD:
                xarm_moving = True
                break
        if xarm_moving:
            self.status_pub.publish(Int8(data=0))
            if self.xarm_collision(): # Check that xArm didn't collide with any structure
                self.status_pub.publish(Int8(data=-1))
                self.status_int = -1
                self.xarm_control = 1
                return
            for asset_type, _ in self.assets.items():
                if asset_type == self.target_asset_xarm: # If object is being carried, ensure gripper hasn't dropped it
                    if self.xarm_carrying:
                        if np.linalg.norm(self.xarm_gripper_position[:3] - self.asset_poses[self.target_asset_xarm][:3]) > self.XARM_CARRY_THRESHOLD:
                            self.status_pub.publish(Int8(data=-1))
                            self.status_int = -1
                            self.xarm_control = 1
                            return
                    continue
                # Check that remaining objects are in proper positions
                if (np.linalg.norm(self.asset_poses[asset_type][:3] - self.initial_asset_poses[asset_type][:3]) > self.OT2_ASSET_POSITION_THRESHOLD) or \
                    (np.linalg.norm(self.asset_poses[asset_type][3:] - self.initial_asset_poses[asset_type][3:]) > self.ASSET_ROTATION_THRESHOLD):
                    self.status_pub.publish(Int8(data=-1))
                    self.status_int = -1
                    self.xarm_control = 1
                    return
        self.get_logger().info("Checked xArm motion")
        # 2. Check if xArm is about to grip something
        if self.target_asset_xarm != "":
            # 2.1: Check if gripper is closing; can use [7] or [8]
            if self.xarm_targets[7] < self.xarm_joints[7]:
                self.status_pub.publish(Int8(data=0))
                self.status_int = 0
                # If gripper is coming from above, check if asset is blocked from above
                if self.xarm_gripper_position[2] > self.asset_poses[self.target_asset_xarm][2] + self.XARM_GRIPPER_THRESHOLD:
                    for asset_type, _ in self.assets.items():
                        if asset_type == self.target_asset_xarm:
                            continue
                        if ((np.linalg.norm(self.asset_poses[asset_type][:2] - self.asset_poses[self.target_asset_xarm][:2]) < self.OT2_ASSET_POSITION_THRESHOLD) and
                            (self.asset_poses[asset_type][2] > self.asset_poses[self.target_asset_xarm][2])):
                            self.status_pub.publish(Int8(data=-1))
                            self.status_int = -1
                            self.xarm_control = 1
                            return
                # If gripper is coming from left, check if asset is blocked from left or from above
                if self.xarm_gripper_position[0] > self.asset_poses[self.target_asset_xarm][0] + self.XARM_GRIPPER_THRESHOLD:
                    for asset_type, _ in self.assets.items():
                        if asset_type == self.target_asset_xarm:
                            continue
                        if (np.linalg.norm(self.asset_poses[asset_type][:2] - self.asset_poses[self.target_asset_xarm][:2]) < self.OT2_ASSET_POSITION_THRESHOLD) or \
                            ((np.linalg.norm(self.asset_poses[asset_type][:2] - self.asset_poses[self.target_asset_xarm][:2]) < self.OT2_ASSET_POSITION_THRESHOLD) and
                             (self.asset_poses[asset_type][2] > self.asset_poses[self.target_asset_xarm][2])):
                            self.status_pub.publish(Int8(data=-1))
                            self.status_int = -1
                            self.xarm_control = 1
                            return
                # If gripper is coming from right, check if asset is blocked from right or from above
                if self.xarm_gripper_position[0] < self.asset_poses[self.target_asset_xarm][0] - self.XARM_GRIPPER_THRESHOLD:
                    for asset_type, _ in self.assets.items():
                        if asset_type == self.target_asset_xarm:
                            continue
                        if (np.linalg.norm(self.asset_poses[asset_type][:2] - self.asset_poses[self.target_asset_xarm][:2]) < self.OT2_ASSET_POSITION_THRESHOLD) or \
                            ((np.linalg.norm(self.asset_poses[asset_type][:2] - self.asset_poses[self.target_asset_xarm][:2]) < self.OT2_ASSET_POSITION_THRESHOLD) and
                             (self.asset_poses[asset_type][2] > self.asset_poses[self.target_asset_xarm][2])):
                            self.status_pub.publish(Int8(data=-1))
                            self.status_int = -1
                            self.xarm_control = 1
                            return
                self.xarm_carrying = True                
                return
             # 2.2: Check if gripper is releasing
            elif self.xarm_targets[7] > self.xarm_joints[7]:
                # Update target object's initial pose
                self.initial_asset_poses[self.target_asset_xarm] = self.asset_poses[self.target_asset_xarm]
                self.xarm_carrying = False
        self.get_logger().info("Checked xArm gripper")
        # 3. Check if OT2 is moving
        ot2_moving = False
        for i in range(len(self.ot2_joints)):
            if abs(self.ot2_joints[i] - self.ot2_targets[i]) > self.OT2_JOINT_THRESHOLD:
                ot2_moving = True
                self.get_logger().info("OT2 is moving")
                break
        if ot2_moving:
            self.status_pub.publish(Int8(data=0))
            self.status_int = 0
            for asset_type, _ in self.assets.items():
                if asset_type == self.target_asset_xarm: # If the object is being carried, ignore it
                    continue
                # Check if assets are aligned with OT2's pipette
                if (np.linalg.norm(self.asset_poses[asset_type][:2] - self.ot2_pipette_position[:2]) > self.OT2_ASSET_POSITION_THRESHOLD) or \
                    (np.linalg.norm(self.asset_poses[asset_type][3:] - self.initial_asset_poses[asset_type][3:]) > self.ASSET_ROTATION_THRESHOLD):
                    self.status_pub.publish(Int8(data=-1))
                    self.status_int = -1
                    self.xarm_control = 1
                    return
                # If pipette is lowering, check if the below asset is not blocked
                if self.ot2_joints[2] - self.ot2_targets[2] > self.OT2_JOINT_THRESHOLD:
                    if asset_type == self.target_asset_ot2:
                        continue
                    if ((np.linalg.norm(self.asset_poses[asset_type][:2] - self.asset_poses[self.target_asset_ot2][:2]) < self.OT2_ASSET_POSITION_THRESHOLD + 0.2) and
                        (self.asset_poses[asset_type][2] > self.asset_poses[self.target_asset_ot2][2])):
                        self.status_pub.publish(Int8(data=-1))
                        self.status_int = -1
                        self.xarm_control = 1
                        return
            return
        self.get_logger().info("Checked OT2 movement")

        # 4. If robots are stationary and everything is fine, publish 1
        self.status_pub.publish(Int8(data=1))
        self.status_int = 1
        self.get_logger().info("All good")

def main():
    rclpy.init()
    safety_checker = SafetyChecker()
    rclpy.spin(safety_checker)
    safety_checker.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()