# Adapted from Isaac Sim examples

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})  # start the simulation app, with GUI open

import omni
import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.prims import Articulation, SingleXFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pxr import Gf
import omni.graph.core as og

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

# Adding assets
add_reference_to_stage(usd_path="assets/xarm6_with_gripper.usd", prim_path="/World/xarm6_with_gripper") # robot
arm = Articulation(prim_paths_expr="/World/xarm6_with_gripper", name="xarm6") # create an articulation object

# loading ot2: absolute path since the usda files reference other files and isaac sim needs to explicitly know where to look
add_reference_to_stage(usd_path="file:///home/sdl1/isaacsim/sdl1-digital-twin/assets/ot2/OT2_inst_no_light.usda", prim_path="/World/ot2")
ot2 = Articulation(prim_paths_expr="/World/ot2", name="ot2")

add_reference_to_stage(usd_path="assets/rectangle.usd", prim_path="/World/prism") # custom rectangular prism object
prism = SingleXFormPrim(prim_path="/World/prism", name="prism")

add_reference_to_stage(usd_path="assets/vial_rack.usd", prim_path="/World/vial_rack") # custom rectangular prism object
vial_rack = SingleXFormPrim(prim_path="/World/vial_rack", name="vial_rack")

add_reference_to_stage(usd_path="assets/vial_20ml.usd", prim_path="/World/vial_20ml") # custom rectangular prism object
vial_20ml = SingleXFormPrim(prim_path="/World/vial_20ml", name="vial_20ml")

# set the initial poses of the arm and the car so they don't collide BEFORE the simulation starts
arm.set_world_poses(positions=np.array([[-0.74, 0.03, 0.0]]) / get_stage_units(), orientations=np.array([[-0.4480736, 0, 0, 0.8939967]]))
ot2.set_world_poses(positions=np.array([[0.0, 0.80, 0.0]]) / get_stage_units(), orientations=np.array([[0.7071068, 0.7071068, 0, 0]]))
prism.set_world_pose(position=np.array([[0.0, -3.0, 0.0]]) / get_stage_units())
vial_rack.set_world_pose(position=np.array([[-6.40, 0.56, 0.054]]) / get_stage_units())
vial_20ml.set_world_pose(position=np.array([[0.0, -2.5, 0.0]]) / get_stage_units())

simulation_app.update()

# For debugging:
#from isaacsim.core.utils.stage import print_stage_prim_paths
#print_stage_prim_paths()

# ActionGraph to echo robot pose from ros topic (real life) into sim
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

# Node subs to FoundationPoseROS2's object pose topic and makes it move accordingly
# Will extend to include all object poses, not just 1
class Sub(Node):
	def __init__(self, object1, object2):
		super().__init__("pose_subscriber")
		self.Y_OFFSET = 0.24
		self.object1 = object1
		self.object2 = object2
		self.timeline = omni.timeline.get_timeline_interface()
		self.ros_world = World(stage_units_in_meters=1.0)
		self.ros_world.scene.add_default_ground_plane()
		self.sub = self.create_subscription(PoseStamped, "/Current_OBJ_position_1", self.cb1, 10) # topic from FoundationPoseROS2
		self.sub = self.create_subscription(PoseStamped, "/Current_OBJ_position_2", self.cb2, 10) # topic from FoundationPoseROS2
		self.cmd_pose1 = np.array([-0.025185562660778385, -0.8522816800404115, 0.11077188243401942, 0.6031173034957719, 0.4053228134277647, -0.5737466978740948, -0.37785931484821517]) # init w to 1, to prevent zero norm quaternion
		self.cmd_pose2 = np.array([0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0]) # init w to 1, to prevent zero norm quaternion
		self.init1 = 0
		self.init2 = 0
		self.object1_initial = np.array([-0.025185562660778385, -0.8522816800404115, 0.11077188243401942, 0.6031173034957719, 0.4053228134277647, -0.5737466978740948, -0.37785931484821517])
		self.object2_initial = np.array([0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0])
		#self.object1_initial = np.array([-0.005226814672087382, -0.8484514705533239, 0.16675297024788074, 0.7007279776439436, 0.11875484793716484, -0.6924162006892137, 0.12424730363900892])
		#self.object2_initial = np.array([-0.19116048041111575, -0.869640728288667, 0.15693135571554453, 0.1596305275838339, 0.9541104785394428, -0.24950245208807353, 0.04404334009337945])
	def cb1(self, msg):
		#if not self.init1:
		#	self.object1_initial = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
		#	self.init1 = 1
		self.cmd_pose1[0], self.cmd_pose1[1], self.cmd_pose1[2] = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
		self.cmd_pose1[3], self.cmd_pose1[4], self.cmd_pose1[5], self.cmd_pose1[6] = msg.pose.orientation.w, msg.pose.orientation.y, msg.pose.orientation.x, msg.pose.orientation.z
	def cb2(self, msg):
		#if not self.init2:
		#	self.object2_initial = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
		#	self.init2 = 1
		self.cmd_pose2[0], self.cmd_pose2[1], self.cmd_pose2[2] = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
		self.cmd_pose2[3], self.cmd_pose2[4], self.cmd_pose2[5], self.cmd_pose2[6] = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
	def run_simulation(self):
		self.timeline.play()
		reset_needed = False
		while simulation_app.is_running():
			rclpy.spin_once(self, timeout_sec=0.0)
			self.ros_world.step(render=True)
			simulation_app.update()
			if self.ros_world.is_stopped() and not reset_needed:
				reset_needed = True
			if self.ros_world.is_playing():
					if reset_needed:
						self.ros_world.reset()
						reset_needed = False
					# y position: 0.24 is an offset; 0.0375 is half the object's width
					# 0.05738 = height of OT2 base from table
					#self.object.set_world_pose(position=np.array([-0.06, -self.cmd_pose[1] - 0.36 - 0.18 + 0.375, 0.05738]), orientation=np.array([0.7071068, 0.7071068, 0, 0]))
					arm.set_world_poses(positions=np.array([[-0.74, 0.03, 0.0]]) / get_stage_units())
					self.object1.set_world_pose(
						position=np.array([self.cmd_pose1[0] - self.object1_initial[0] + 0.07, -self.cmd_pose1[1] - self.Y_OFFSET + 0.0375, self.cmd_pose1[2] - self.object1_initial[2] + 0.05738]),
						orientation=np.array([self.cmd_pose1[3] - self.object1_initial[3] + 0.7071068, self.cmd_pose1[4] - self.object1_initial[4] + 0.7071068, self.cmd_pose1[5] - self.object1_initial[5], self.cmd_pose1[6] - self.object1_initial[6]])
					)
					self.object2.set_world_pose(
						position=np.array([self.cmd_pose2[0] - self.object2_initial[0] - 0.0135, -self.cmd_pose2[1] - self.Y_OFFSET, self.cmd_pose2[2] - self.object2_initial[2] + 0.05738]),
						orientation=np.array([self.cmd_pose2[3] - self.object2_initial[3] + 1, self.cmd_pose2[4] - self.object2_initial[4], self.cmd_pose2[5] - self.object2_initial[5], self.cmd_pose2[6] - self.object2_initial[6]])
					)
		self.timeline.stop()
		self.destroy_node()
		simulation_app.close()

if __name__ == "__main__":
	rclpy.init()
	subscriber = Sub(prism, vial_20ml) # will extend to multiple objects in main subscriber
	subscriber.run_simulation()