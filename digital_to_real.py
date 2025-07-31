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
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
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

# loading xarm
add_reference_to_stage(usd_path="assets/xarm6_with_gripper.usd", prim_path="/World/xarm6_with_gripper") # robot
arm = Articulation(prim_paths_expr="/World/xarm6_with_gripper", name="xarm6") # create an articulation object
# loading ot2: absolute path since the usda files reference other files and isaac sim needs to explicitly know where to look
add_reference_to_stage(usd_path="file:///home/sdl1/isaacsim/sdl1-digital-twin/assets/ot2/OT2_inst_no_light.usda", prim_path="/World/ot2")
simulation_app.update()
world.reset()
simulation_app.update()
ot2 = SingleArticulation(prim_path="/World/ot2", name="ot2")
ot2.initialize()
simulation_app.update()

arm.set_world_poses(positions=np.array([[-0.74, 0.03, 0.0]]) / get_stage_units(), orientations=np.array([[-0.4480736, 0, 0, 0.8939967]]))
ot2.set_world_poses(positions=np.array([[0.0, 0.80, 0.0]]) / get_stage_units(), orientations=np.array([[0.7071068, 0.7071068, 0, 0]]))
simulation_app.update()

# 'PrismaticJointMiddleBar', 'PrismaticJointPipetteHolder', 'PrismaticJointLeftPipette', 'PrismaticJointRightPipette']
#  more negative = more front,	more negative = more left,    more negative = more down,   more negative = more down
# [-0.08,0.2] #38 cm			[-0.19, 0.18] #42 cm					[-0.1, 0]					  [-0.1, 0]				
# pipette dims: 80x130

class SimulatedWorld(Node):
	def __init__(self):
		super().__init__("pose_subscriber")
		self.timeline = omni.timeline.get_timeline_interface()
		self.ros_world = World(stage_units_in_meters=1.0)
		self.ros_world.scene.add_default_ground_plane()
		self.ot2_sub = self.create_subscription(JointState, "/sim_ot2/target_joint_states", self.ot2_callback, 10)
		self.ot2_joint_targets = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
		self.xarm_sub = self.create_subscription(JointState, "/sim_xarm/target_joint_states", self.xarm_callback, 10)
		self.xarm_joint_targets = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
		self.ot2_pub = self.create_publisher(JointState, "/sim_ot2/joint_states", 10)
		self.xarm_pub = self.create_publisher(JointState, "/sim_xarm/joint_states", 10)
		self.world_pub = self.create_publisher(Float32MultiArray, "/sim_world/poses", 10)
		# /sim_world/poses: [arm1, ..., arm7, ot2_1, ..., ot2_5]
		self.ot2_joints = [String(joint) for joint in ot2.dof_names]
		self.xarm_joints = [String(joint) for joint in ot2.dof_names]
		self.dim = MultiArrayDimension(label="joints", size=12, stride=12)
	def ot2_callback(self, msg):
		self.ot2_joint_targets = np.array(msg.position)
	def xarm_callback(self, msg):
		self.xarm_joint_targets = np.array(msg.position)
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
					ot2.set_joint_position_targets(self.ot2_joint_targets)
					arm.set_joint_position_targets(self.xarm_joint_targets)
					self.world_pub.publish(Float32MultiArray(layout=MultiArrayLayout(dim=self.dim, data_offset=0), data=np.concatenate((self.get_joint_positions(arm), self.get_joint_positions(ot2)))))
		self.timeline.stop()
		self.destroy_node()
		simulation_app.close()

if __name__ == "__main__":
	rclpy.init()
	simulated_world = SimulatedWorld()
	simulated_world.run_simulation()