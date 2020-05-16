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


human = p.loadURDF("humoro/humoro/data/human.urdf", [0, 0, 0])


position, orientation = p.getBasePositionAndOrientation(human)
orientation

p.getNumJoints(human)

joint_index = 2
joint_info = p.getJointInfo(human, joint_index)
name, joint_type, lower_limit, upper_limit = \
	joint_info[1], joint_info[2], joint_info[8], joint_info[9]
name, joint_type, lower_limit, upper_limit

joint_positions = [j[0] for j in p.getJointStates(human, range(6))]
joint_positions

world_position, world_orientation = p.getLinkState(human, 2)[:2]
world_position

# p.setGravity(0, 0, -9.81)
p.setTimeStep(0.0001)
p.setRealTimeSimulation(0)


for _ in range(100000):
	p.stepSimulation()


