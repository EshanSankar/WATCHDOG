import numpy as np
import json

OT2_OFFSETS = {"x": -0.12756, "y": -0.17065, "z": 0.054}
OT2_COORDS = {
    1: {"x": 0.0, "y": 0.0}, 2: {"x": 0.13, "y": 0.0}, 3: {"x": 0.26, "y": 0.0},
    4: {"x": 0.0, "y": 0.09}, 5: {"x": 0.13, "y": 0.09}, 6: {"x": 0.26, "y": 0.09},
    7: {"x": 0.0, "y": 0.18}, 8: {"x": 0.13, "y": 0.18}, 9: {"x": 0.26, "y": 0.18},
    10: {"x": 0.0, "y": 0.27}, 11: {"x": 0.13, "y": 0.27}, 12: {"x": 0.26, "y": 0.27}}

class Asset():
	def __init__(self, workflow_path):
		self.slot = 0
		self.type = ""
		self.initial_position = np.array([0, 0, 0])
		self.initial_orientation = np.array([1, 0, 0, 0])
		self.above = ""
		self.asset_path = ""
		self.simulated_asset = None
		self.sub = {}
		self.pub = {}
		self.cmd_pose = {}
		self.initial_tracked_position = {}
		self.stationary = True
		with open(f"{workflow_path}", "r") as f:
			workflow = json.load(f)
			for asset_type, asset in workflow["global_config"]["labware"].items():
				self.type = asset_type
				self.slot = asset["slot"]
				self.initial_position = np.array([
					OT2_COORDS[self.slot]["x"] + OT2_OFFSETS["x"] + asset["offsets"]["x"],
					OT2_COORDS[self.slot]["y"] + OT2_OFFSETS["y"] + asset["offsets"]["z"],
					OT2_OFFSETS["z"] + asset["offsets"]["y"]])
				self.initial_orientation = np.array([
					asset["offsets"]["w"],
					asset["offsets"]["rx"],
					asset["offsets"]["ry"],
					asset["offsets"]["rz"]])
				self.above = asset["above"]
				self.asset_path = asset["asset_path"]
				self.stationary = asset["stationary"]

class Assets():
	def __init__(self, workflow_path):
		self.asset_dict = {}
		with open(f"{workflow_path}", "r") as f:
			workflow = json.load(f)
			for asset_type, asset in workflow["global_config"]["labware"].items():
				self.asset_dict[asset_type] = Asset(workflow_path)