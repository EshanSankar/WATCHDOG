from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})  # start the simulation app, with GUI open

import omni
import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.prims import Articulation, SingleXFormPrim, SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.types import ArticulationAction

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension, MultiArrayLayout, Float32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from pxr import Gf
import omni.graph.core as og
import json

# preparing the scene
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()  # add ground plane
set_camera_view(
    eye=[5.0, 0.0, 1.5], target=[0.00, 0.00, 1.00], camera_prim_path="/OmniverseKit_Persp"
)  # set camera view

# loading assets
ASSET_PATH = "file:///home/sdl1/isaacsim/sdl1-digital-twin/assets"
WORKFLOW_PATH = "video_workflow.json"

# loading xarm
add_reference_to_stage(usd_path=f"{ASSET_PATH}/xarm6_with_gripper.usd", prim_path="/World/xarm6_with_gripper") # robot
xarm = Articulation(prim_paths_expr="/World/xarm6_with_gripper", name="xarm6") # create an articulation object
# loading ot2: absolute path since the usda files reference other files and isaac sim needs to explicitly know where to look
add_reference_to_stage(usd_path=f"{ASSET_PATH}/ot2/OT2_inst_no_light.usda", prim_path="/World/ot2")
xarm_gripper_center = SingleXFormPrim("/World/xarm6_with_gripper/xarm6_with_gripper/link_tcp")
simulation_app.update()
world.reset()
simulation_app.update()
ot2 = SingleArticulation(prim_path="/World/ot2", name="ot2")
ot2.initialize()
simulation_app.update()
xarm.set_world_poses(positions=np.array([[2, 2, 0.0]]) / 1.0, orientations=np.array([[-0.4480736, 0, 0, 0.8939967]]))
ot2.set_world_pose(position=np.array([[0.0, 0.0, 0.0]]) / 1.0, orientation=np.array([[0.7071068, 0.7071068, 0, 0]]))
simulation_app.update()
xarm.set_joint_positions(positions=np.array([-2.87, -0.2179, -0.7428, -1.13558, 1.4665, -3.1397, 0.3, 0.3]), joint_indices=np.array([0, 1, 2, 3, 4, 5, 10, 11]))
xarm.set_world_poses(positions=np.array([[-0.525, -0.33, 0.0]]) / 1.0, orientations=np.array([[0.7071068, 0, 0, -0.7071068]]))
simulation_app.update()

OT2_BASE_HEIGHT = 0.054
OT2_OFFSETS = {"x": -0.12756, "y": -0.17065}
OT2_COORDS = {
	1: [0.0, 0.0], 2: [0.13, 0.0], 3: [0.26, 0.0],
	4: [0.0, 0.09], 5: [0.13, 0.09], 6: [0.26, 0.09],
	7: [0.0, 0.18], 8: [0.13, 0.18], 9: [0.26, 0.18],
	10: [0.0, 0.27], 11: [0.13, 0.27], 12: [0.26, 0.27]}

# 'PrismaticJointMiddleBar', 'PrismaticJointPipetteHolder', 'PrismaticJointLeftPipette', 'PrismaticJointRightPipette']
#  more negative = more front,	more negative = more left,    more negative = more down,   more negative = more down
# [-0.08,0.2] #38 cm			[-0.19, 0.18] #42 cm					[-0.1, 0]					  [-0.1, 0]				
# pipette dims: 80x130
OTHER_ASSETS = {"vial_1": [7, -0.046, 0.022, 0.037], "vial_2": [7, 0.046, -0.022, 0.037], "vial_rack_lid": [7, -0.0635, -0.0425, 0.065]}

def LoadAssets(asset_path=ASSET_PATH):
	assets = {}
	with open(f"{WORKFLOW_PATH}", "r") as f:
		workflow = json.load(f)
		for asset_type, asset in workflow["global_config"]["labware"].items():
			# Below is the default. For now, manually specify paths since asset names aren't yet standardized
			#add_reference_to_stage(usd_path=f"{asset_path}/{asset_type}/{asset_type}.usda", prim_path=f"/World/{asset_type}")
			if asset_type == "tip_rack":
				add_reference_to_stage(usd_path=f"{asset_path}/{asset_type}/tiprack_inst.usda", prim_path=f"/World/{asset_type}")
			elif asset_type == "reactor":
				add_reference_to_stage(usd_path=f"{asset_path}/{asset_type}/tiprack_inst.usda", prim_path=f"/World/{asset_type}")
			elif asset_type == "vial_rack":
				add_reference_to_stage(usd_path=f"{asset_path}/{asset_type}/vial_rack.usdc", prim_path=f"/World/{asset_type}")
			assets[asset_type] = [asset["slot"], SingleXFormPrim(prim_path=f"/World/{asset_type}", name=asset_type)]
			# Will also need to specify default rotation per asset as those are also not standardized
			assets[asset_type][1].set_world_pose(position=np.array([[
				OT2_COORDS[asset["slot"]][0] + OT2_OFFSETS["x"],
				OT2_COORDS[asset["slot"]][1] + OT2_OFFSETS["y"],
				OT2_BASE_HEIGHT]]) / 1.0, orientation=np.array([1, 0, 0, 0])) # get_stage_units() -> 1.0
			if asset_type == "tip_rack" or asset_type == "reactor":
				assets[asset_type][1].set_world_pose(orientation=np.array([0.7071068, 0, 0, -0.7071068]))
			# Because of non-standard assets, some assets will have their origins at their (0,0) instead of the center of the asset
			# Since the above calculation is done w/ r.t. the center of the asset, will need to offset
			if asset_type == "vial_rack":
				assets[asset_type][1].set_world_pose(position=np.array([[
				OT2_COORDS[asset["slot"]][0] + OT2_OFFSETS["x"] - 0.0635,
				OT2_COORDS[asset["slot"]][1] + OT2_OFFSETS["y"] - 0.0425,
				OT2_BASE_HEIGHT]]))
	for asset_type, offset in OTHER_ASSETS.items():
		if asset_type == "vial_1" or asset_type == "vial_2":
			add_reference_to_stage(usd_path=f"{asset_path}/{asset_type}/vial_20ml_inst.usda", prim_path=f"/World/{asset_type}")
		if asset_type == "vial_rack_lid":
			add_reference_to_stage(usd_path=f"{asset_path}/{asset_type}/vial_rack_lid.usda", prim_path=f"/World/{asset_type}")
		assets[asset_type] = [offset[0], SingleXFormPrim(prim_path=f"/World/{asset_type}", name=asset_type)]
		assets[asset_type][1].set_world_pose(position=np.array([[
			OT2_COORDS[offset[0]][0] + OT2_OFFSETS["x"] + offset[1],
			OT2_COORDS[offset[0]][1] + OT2_OFFSETS["y"] + offset[2],
			OT2_BASE_HEIGHT + offset[3]]]) / 1.0)
	return assets

class SimulatedWorld(Node):
	def __init__(self):
		super().__init__("pose_subscriber")
		self.timeline = omni.timeline.get_timeline_interface()
		self.ros_world = World(stage_units_in_meters=1.0)
		self.ros_world.scene.add_default_ground_plane()
		self.ot2_sub = self.create_subscription(JointState, "/sim_ot2/target_joint_states", self.ot2_cb, 10)
		self.ot2_joint_targets = np.array([0.0, 0.0, 0.0])
		self.xarm_sub = self.create_subscription(JointState, "/sim_xarm/target_joint_states", self.xarm_cb, 10)
		self.xarm_gripper_position_sub = self.create_subscription(Float32, "/orchestrator/gripper_position", self.xarm_gripper_position_cb, 10)
		self.xarm_gripper_position = 500.0
		self.ot2_joints = ["PrismaticJointMiddleBar", "PrismaticJointPipetteHolder", "PrismaticJointRightPipette"]
		self.ot2_joint_indices = np.array([0, 1, 3])
		self.xarm_joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "left_finger_joint", "right_finger_joint"]
		self.xarm_joint_indices = np.array([0, 1, 2, 3, 4, 5, 10, 11])
		# gripper joint limits [0, 0.85]; real is [0, 850]
		self.xarm_joint_targets = np.array([-2.87, -0.2179, -0.7428, -1.13558, 1.4665, -3.1397, 0.3, 0.3])
		# /sim_robot/joint_states: [[joint1, ..., jointN], [joint1_goal, ..., jointN_goal]]
		self.ot2_pub = self.create_publisher(Float32MultiArray, "/sim_ot2/joint_states", 10)
		self.xarm_pub = self.create_publisher(Float32MultiArray, "/sim_xarm/joint_states", 10)
		self.xarm_gripper_position_pub = self.create_publisher(Float32MultiArray, "/sim_xarm/gripper_position", 10)
		self.xarm_control_joints = np.array([2.865, -0.0861, -0.4979, 4.3559, 1.3179, 1.021, 0.5, 0.5])
		self.xarm_control_sub = self.create_subscription(JointState, "/xarm/joint_states", self.xarm_control_cb, 10)
		ot2.set_joint_positions(positions=self.ot2_joint_targets, joint_indices=self.ot2_joint_indices)
		xarm.set_joint_positions(positions=self.xarm_joint_targets, joint_indices=self.xarm_joint_indices)
		self.ot2_dim = [MultiArrayDimension(label="joints", size=2, stride=6), MultiArrayDimension(label="goals", size=3, stride=3)]
		self.xarm_dim = [MultiArrayDimension(label="joints", size=2, stride=16), MultiArrayDimension(label="goals", size=8, stride=8)]
		self.xarm_gripper_dim = [MultiArrayDimension(label="pose", size=7, stride=7)]
		self.asset_dim = [MultiArrayDimension(label="pose", size=7, stride=7)]
		self.safety = 1
		self.c = 0
		self.safety_sub = self.create_subscription(String, "/safety_checker/status_int", self.safety_cb, 10)
		# object pose subscribers
		self.assets = LoadAssets()
		simulation_app.update()
		self.asset_subs = {}
		self.asset_pubs = {}
		self.asset_cmd_poses = {}
		self.asset_initial_poses = {}
		self.asset_initial_tracked_poses = {}
		self.initialized_assets = {}
		for asset_type, asset in self.assets.items():
			# will manually skip some stationary assets
			if asset_type == "tip_rack" or asset_type == "reactor" or asset_type == "vial_rack":
				continue
			self.asset_subs[asset_type] = self.create_subscription(PoseStamped, f"/asset_position_{asset_type}", lambda msg, asset_type=asset_type: self.asset_cb(msg, asset_type), 10)
			self.asset_cmd_poses[asset_type] = np.concatenate((asset[1].get_world_pose()[0], asset[1].get_world_pose()[1]), axis=0)
			self.asset_initial_poses[asset_type] = self.asset_cmd_poses[asset_type].copy()
			self.asset_pubs[asset_type] = self.create_publisher(Float32MultiArray, f"/sim_{asset_type}/pose", 10)
			self.initialized_assets[asset_type] = False
	def asset_cb(self, msg, asset_type):
		tracked_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
		if not self.initialized_assets[asset_type]:
			self.asset_initial_tracked_poses[asset_type] = tracked_pose
			self.initialized_assets[asset_type] = True
		else:
			# Bug with FoundationPose causes it to track the cylindrical vials as constantly rotating along the z-axis
			# Due to this limitation, will exclude orientation tracking for vials only
			if asset_type == "vial_1" or asset_type == "vial_2":
				self.asset_cmd_poses[asset_type][:3] = np.add(self.asset_initial_poses[asset_type][:3], np.subtract(tracked_pose[:3], self.asset_initial_tracked_poses[asset_type][:3]))
			else:
				self.asset_cmd_poses[asset_type] = np.add(self.asset_initial_poses[asset_type], np.subtract(tracked_pose, self.asset_initial_tracked_poses[asset_type]))
	def ot2_cb(self, msg):
		self.ot2_joint_targets = msg.position
	def xarm_cb(self, msg):
		# msg: [joint1, joint2, joint3, joint4, joint5, joint6, relative] OR [left_finger, right_finger]
		if len(msg.position) == 7:
			self.xarm_joint_targets = np.concatenate((np.array(msg.position)[:6] if msg.position[6] == 0.0 else np.add(np.array(msg.position)[:6], xarm.get_joint_positions()[0][:6]), self.xarm_joint_targets[6:]))
		else:
			self.xarm_joint_targets = np.concatenate((self.xarm_joint_targets[:6], np.array(msg.position)))
	def xarm_gripper_position_cb(self, msg):
		self.xarm_gripper_position = msg.data / 1000
	def safety_cb(self, msg):
		self.safety = int(msg.data)
	def xarm_control_cb(self, msg):
		self.xarm_control_joints = np.array(msg.position)
	def run_simulation(self):
		self.timeline.play()
		reset_needed = False
		while simulation_app.is_running():
			self.c += 1
			rclpy.spin_once(self, timeout_sec=0.0)
			self.ros_world.step(render=True)
			simulation_app.update()
			if self.ros_world.is_stopped() and not reset_needed:
				reset_needed = True
			if self.ros_world.is_playing():
					if reset_needed:
						self.ros_world.reset()
						reset_needed = False
					if self.safety == -1:
					#if self.c > 400 and self.c < 600: (for testing)
						xarm.set_joint_positions(positions=np.concatenate((self.xarm_control_joints, np.array([self.xarm_gripper_position, self.xarm_gripper_position]))), joint_indices=self.xarm_joint_indices)
						for asset_type, _ in self.assets.items():
							self.assets[asset_type][1].set_world_pose(position=self.asset_cmd_poses[asset_type][:3], orientation=self.asset_cmd_poses[asset_type][3:])
							self.asset_pubs[asset_type].publish(Float32MultiArray(layout=MultiArrayLayout(dim=self.asset_dim, data_offset=0), data=self.asset_cmd_poses[asset_type]))
						self.xarm_joint_targets[:6] = self.xarm_control_joints
						continue
					# move the robots
					for i in range(3): # individually move the OT2 DOF
						ot2.apply_action(ArticulationAction(joint_positions=self.ot2_joint_targets[i:i+1], joint_indices=self.ot2_joint_indices[i:i+1]))
						self.ot2_pub.publish(Float32MultiArray(layout=MultiArrayLayout(dim=self.ot2_dim, data_offset=0), data=np.concatenate((ot2.get_joint_positions()[:3], self.ot2_joint_targets))))
					xarm.set_joint_position_targets(positions=self.xarm_joint_targets, joint_indices=self.xarm_joint_indices)
					#print(xarm_gripper_center.get_world_pose())
					self.ot2_pub.publish(Float32MultiArray(layout=MultiArrayLayout(dim=self.ot2_dim, data_offset=0), data=np.concatenate((ot2.get_joint_positions()[:3], self.ot2_joint_targets))))
					self.xarm_pub.publish(Float32MultiArray(layout=MultiArrayLayout(dim=self.xarm_dim, data_offset=0), data=np.concatenate((xarm.get_joint_positions()[0][:6], xarm.get_joint_positions()[0][10:], self.xarm_joint_targets))))
					self.xarm_gripper_position_pub.publish(Float32MultiArray(layout=MultiArrayLayout(dim=self.xarm_gripper_dim, data_offset=0), data=np.concatenate((xarm_gripper_center.get_world_pose()[0], xarm_gripper_center.get_world_pose()[1]))))
					# mirror asset positions from real-world for non-stationary assets
					for asset_type, _ in self.asset_pubs.items():
						self.assets[asset_type][1].set_world_pose(position=self.asset_cmd_poses[asset_type][:3], orientation=self.asset_cmd_poses[asset_type][3:])
						self.asset_pubs[asset_type].publish(Float32MultiArray(layout=MultiArrayLayout(dim=self.asset_dim, data_offset=0), data=self.asset_cmd_poses[asset_type]))
		self.timeline.stop()
		self.destroy_node()
		simulation_app.close()

if __name__ == "__main__":
	rclpy.init()
	simulated_world = SimulatedWorld()
	simulated_world.run_simulation()