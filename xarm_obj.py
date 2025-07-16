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

# set the initial poses of the arm and the car so they don't collide BEFORE the simulation starts
arm.set_world_poses(positions=np.array([[3.0, 3.0, 0.0]]) / get_stage_units())
ot2.set_world_poses(positions=np.array([[0.0, 0.80, 0.0]]) / get_stage_units(), orientations=np.array([[0.7071068, 0.7071068, 0, 0]]))
prism.set_world_pose(position=np.array([[0.0, -3.0, 0.0]]) / get_stage_units())

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
	def __init__(self, object):
		super().__init__("pose_subscriber")
		self.object = object
		self.timeline = omni.timeline.get_timeline_interface()
		self.ros_world = World(stage_units_in_meters=1.0)
		self.ros_world.scene.add_default_ground_plane()
		self.sub = self.create_subscription(PoseStamped, "/Current_OBJ_position_1", self.cb, 10) # topic from FoundationPoseROS2
		self.cmd_pose = np.array([0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0]) # init w to 1, to prevent zero norm quaternion
	def cb(self, msg):
		self.cmd_pose[0], self.cmd_pose[1], self.cmd_pose[2] = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
		self.cmd_pose[3], self.cmd_pose[4], self.cmd_pose[5], self.cmd_pose[6] = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
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
					self.object.set_world_pose(position=np.array([-0.06, -self.cmd_pose[1] - 0.24 + 0.0375, 0.05738]), orientation=np.array([0.7071068, 0.7071068, 0, 0]))
		self.timeline.stop()
		self.destroy_node()
		simulation_app.close()

if __name__ == "__main__":
	rclpy.init()
	subscriber = Sub(prism) # will extend to multiple objects in main subscriber
	subscriber.run_simulation()