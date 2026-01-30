# ik/inspect_urdf.py
from yourdfpy import URDF

robot = URDF.load("so101.urdf")
print("Links:", list(robot.link_map.keys())[-10:])
print("Joints:", robot.joint_names)
