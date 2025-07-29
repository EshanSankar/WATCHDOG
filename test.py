# Adapted from Isaac Sim examples

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

# loading ot2: absolute path since the usda files reference other files and isaac sim needs to explicitly know where to look
add_reference_to_stage(usd_path="file:///home/sdl1/isaacsim/sdl1-digital-twin/assets/ot2/OT2_inst_no_light.usda", prim_path="/World/ot2")
simulation_app.update()
world.reset()
simulation_app.update()
ot2 = SingleArticulation(prim_path="/World/ot2", name="ot2")
ot2.initialize()
simulation_app.update()

# 'PrismaticJointMiddleBar', 'PrismaticJointPipetteHolder', 'PrismaticJointLeftPipette', 'PrismaticJointRightPipette']
#  more negative = more front,	more negative = more left,    more negative = more down,   more negative = more down
# [-0.08,0.2] #38 cm			[-0.19, 0.18] #42 cm					[-0.1, 0]					  [-0.1, 0]				
# pipette dims: 80x130

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
			("SubscribeJointStateArm.inputs:topicName", "/ot2/joint_states"),
			("ArticulationControllerArm.inputs:targetPrim", "/World/ot2/ot2/root_joint")
		],
	},
	)
except Exception as e:
	print(e)

simulation_app.update()

class Sub(Node):
	def __init__(self):
		super().__init__("pose_subscriber")
		self.timeline = omni.timeline.get_timeline_interface()
		self.ros_world = World(stage_units_in_meters=1.0)
		self.ros_world.scene.add_default_ground_plane()
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
		self.timeline.stop()
		self.destroy_node()
		simulation_app.close()

if __name__ == "__main__":
	rclpy.init()
	subscriber = Sub()
	subscriber.run_simulation()
