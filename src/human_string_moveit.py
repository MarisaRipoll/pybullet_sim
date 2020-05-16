import argparse
import os

import pybullet as p
import pybullet_data
import math
import time

p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")


human = p.loadURDF("human.urdf", [0, 0, 1], useFixedBase=1)


# position, orientation = p.getBasePositionAndOrientation(human)
# orientation

num = p.getNumJoints(human)
print("The number of Joints is: ", num)

"""
for i in range(num):
	info = p.getJointInfo(human, i)
	print(info)"""
info = p.getJointInfo(human, 25)
print(info)

# joint_index = 2
# joint_info = p.getJointInfo(human, joint_index)
# name, joint_type, lower_limit, upper_limit = \
# 	joint_info[1], joint_info[2], joint_info[8], joint_info[9]
# name, joint_type, lower_limit, upper_limit

# joint_positions = [j[0] for j in p.getJointStates(human, range(6))]
# joint_positions

# world_position, world_orientation = p.getLinkState(human, 2)[:2]
# world_position

# p.setGravity(0, 0, -9.81)
p.setTimeStep(-0.0001)
p.setRealTimeSimulation(0)
color = [1, 0, 0]

"""Joint indeces:
0: Waist
1: Between Feet (in floor)
4: Upper Chest
7: Neck
8: Head
21: left_shoulder
22: left_elbow
25: left_wrist_2 (out of 3) """


p.addUserDebugLine([0,0,0],
                   [0.75,0,0.75],
                   color,
                   3,
		   parentObjectUniqueId=human,
		   parentLinkIndex = 25)


# p.setJointMotorControlArray(human, jointIndices=(21, 22, 25), controlMode = p.POSITION_CONTROL)

for _ in range(10000000000):
	p.stepSimulation()

