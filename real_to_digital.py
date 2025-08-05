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
from safety_checker import Assets

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
ORCHESTRATOR_PATH = "file:///home/sdl1/ros2_projects/digital_twin/src/catalyst-OT2-xArm"
# loading xarm
add_reference_to_stage(usd_path=f"{ASSET_PATH}/xarm6_with_gripper.usd", prim_path="/World/xarm6_with_gripper") # robot
xarm = Articulation(prim_paths_expr="/World/xarm6_with_gripper", name="xarm6") # create an articulation object
# loading ot2: absolute path since the usda files reference other files and isaac sim needs to explicitly know where to look
add_reference_to_stage(usd_path=f"{ASSET_PATH}/ot2/OT2_inst_no_light.usda", prim_path="/World/ot2")
simulation_app.update()
world.reset()
simulation_app.update()
ot2 = SingleArticulation(prim_path="/World/ot2", name="ot2")
ot2.initialize()
simulation_app.update()
xarm.set_world_poses(positions=np.array([[-0.74, 0.03, 0.0]]) / get_stage_units(), orientations=np.array([[-0.4480736, 0, 0, 0.8939967]]))
ot2.set_world_pose(position=np.array([[0.0, 0.80, 0.0]]) / get_stage_units(), orientation=np.array([[0.7071068, 0.7071068, 0, 0]]))
simulation_app.update()
# Loading custom labware from workflow json
class Assets():
	def __init__(self, asset_path):
		self.assets = {}
		with open(f"{ORCHESTRATOR_PATH}/xarm_workflow.json", "r") as f:
			workflow = json.load(f)
			for asset in workflow["global_config"]["labware"]:
				print(asset)
				add_reference_to_stage(usd_path=f"{asset_path}/{asset}.usd", prim_path=f"/World/{asset}")
				self.assets[f"{asset}"] = [asset["slot"], SingleXFormPrim(prim_path=f"/World/{asset}", name=f"{asset}")]
				self.assets[f"{asset}"][1].set_world_pose(position=np.array([[
					OT2_COORDS[asset["slot"]][0] + OT2_SLOT_DIMS["x"],
					OT2_COORDS[asset["slot"]][1] + OT2_SLOT_DIMS["y"],
					OT2_BASE_HEIGHT]]) / get_stage_units())
OT2_BASE_HEIGHT = 0.05738
OT2_SLOT_DIMS = {"x": 0.04, "y": 0.025}
OT2_COORDS = {
	1: (0.0, 0.0), 2: (0.13, 0.0), 3: (0.26, 0.0),
	4: (0.0, 0.09), 5: (0.13, 0.09), 6: (0.26, 0.09),
	7: (0.0, 0.18), 8: (0.13, 0.18), 9: (0.26, 0.18),
	10: (0.0, 0.27), 11: (0.13, 0.27), 12: (0.26, 0.27)}

# ActionGraph to echo xArm pose from ros topic (real life) into sim
try:
	og.Controller.edit(
	{"graph_path": "/World/ActionGraph", "evaluator_name": "execution"},
	{
		og.Controller.Keys.CREATE_NODES: [
			("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
			("Context", "isaacsim.ros2.bridge.ROS2Context"),
			("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
			("SubscribeJointStateArm", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
			("ArticulationControllerArm", "isaacsim.core.nodes.IsaacArticulationController")
		],
		og.Controller.Keys.CONNECT: [
			("OnPlaybackTick.outputs:tick", "SubscribeJointStateArm.inputs:execIn"),
			("OnPlaybackTick.outputs:tick", "ArticulationControllerArm.inputs:execIn"),
			("SubscribeJointStateArm.outputs:effortCommand", "ArticulationControllerArm.inputs:effortCommand"),
			("SubscribeJointStateArm.outputs:jointNames", "ArticulationControllerArm.inputs:jointNames"),
			("SubscribeJointStateArm.outputs:positionCommand", "ArticulationControllerArm.inputs:positionCommand"),
			("SubscribeJointStateArm.outputs:velocityCommand", "ArticulationControllerArm.inputs:velocityCommand"),
			("Context.outputs:context", "SubscribeJointStateArm.inputs:context")
		],
		og.Controller.Keys.SET_VALUES: [
			("SubscribeJointStateArm.inputs:topicName", "/xarm/joint_states"),
			("ArticulationControllerArm.inputs:targetPrim", "/World/xarm6_with_gripper/xarm6_with_gripper/root_joint")
		],
	},
	)
except Exception as e:
	print(e)
simulation_app.update()

class SimulatedWorld(Node):
	def __init__(self):
		super().__init__("pose_subscriber")
		self.timeline = omni.timeline.get_timeline_interface()
		self.ros_world = World(stage_units_in_meters=1.0)
		self.ros_world.scene.add_default_ground_plane()
		self.ot2_sub = self.create_subscription(JointState, "/sim_ot2/target_joint_states", self.ot2_cb, 10)
		#self.ot2_joint_targets = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
		self.ot2_joint_targets = np.array([0.0, 0.0])
		self.ot2_pub = self.create_publisher(Float32MultiArray, "/sim_ot2/joint_states", 10)
		self.ot2_joints = ["PrismaticJointMiddleBar", "PrismaticJointPipetteHolder"]
		self.ot2_joint_indices = np.array([0, 1])
		self.asset_dim = [MultiArrayDimension(label="pose", size=7, stride=7)]
		self.safety = 1
		self.safety_sub = self.create_subscription(String, "/safety_checker/status_int", self.safety_cb, 10)
		# object pose subscribers
		self.assets = Assets()
		simulation_app.update()
		self.asset_subs = {}
		self.asset_pubs = {}
		self.asset_cmd_poses = {}
		self.asset_initial_poses = {}
		for asset in self.assets:
			self.asset_subs[f"{asset}"] = self.create_subscription(PoseStamped, f"/asset_position_{self.assets[{asset}][0]}", lambda msg: self.asset_cb(msg, f"{asset}"), 10)
			self.asset_cmd_poses[f"{asset}"] = np.concatenate(self.assets[f"{asset}"][1].get_world_pose()[0], self.assets[f"{asset}"][1].get_world_pose()[1], axis=1)
			self.asset_initial_poses[f"{asset}"] = self.asset_cmd_poses[f"{asset}"].copy()
			self.asset_pubs[f"{asset}"] = self.create_publisher(Float32MultiArray, f"/sim_{asset}/pose", 10)
	def asset_cb(self, msg, asset):
		self.asset_cmd_poses[f"{asset}"][0], self.asset_cmd_poses[f"{asset}"][1], self.asset_cmd_poses[f"{asset}"][2] = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
		self.asset_cmd_poses[f"{asset}"][3], self.asset_cmd_poses[f"{asset}"][4], self.asset_cmd_poses[f"{asset}"][5], self.asset_cmd_poses[f"{asset}"][6] = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
		self.asset_cmd_poses[f"{asset}"] = np.substract(self.asset_cmd_poses[f"{asset}"], self.asset_initial_poses[f"{asset}"])
	def ot2_cb(self, msg):
		self.ot2_joint_targets = msg.position
		self.ot2_reached_goal = False
	def safety_cb(self, msg):
		self.safety = int(msg.data)
	def run_simulation(self):
		self.timeline.play()
		reset_needed = False
		while simulation_app.is_running():
			rclpy.spin_once(self, timeout_sec=0.0)
			if self.safety == -1:
				continue # pause the sim if unsafe
			self.ros_world.step(render=True)
			simulation_app.update()
			if self.ros_world.is_stopped() and not reset_needed:
				reset_needed = True
			if self.ros_world.is_playing():
					if reset_needed:
						self.ros_world.reset()
						reset_needed = False
					# move only the OT2
					ot2.apply_action(ArticulationAction(joint_positions=self.ot2_joint_targets, joint_indices=self.ot2_joint_indices))
					# mirror asset positions from real-world
					for asset in self.assets:
						self.assets[f"{asset}"][1].set_world_pose(position=self.asset_cmd_poses[f"{asset}"][:3], orientation=self.asset_cmd_poses[f"{asset}"][3:])
						self.asset_pubs[f"{asset}"].publish(Float32MultiArray(layout=MultiArrayLayout(dim=self.asset_dim, data_offset=0), data=self.asset_cmd_poses[f"{asset}"]))
		self.timeline.stop()
		self.destroy_node()
		simulation_app.close()

if __name__ == "__main__":
	rclpy.init()
	simulated_world = SimulatedWorld()
	simulated_world.run_simulation()