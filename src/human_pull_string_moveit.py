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

joints = p.getNumJoints(human)
print("The number of Joints is: ", joints)

""""""
for i in range(joints):
	info = p.getJointInfo(human, i)
	print(info)
info = p.getJointInfo(human, 25)
print(info)

cons = p.getNumConstraints()
print("The number of Constraints is: ", cons)


for i in range(cons):
	info = p.getConstraintInfo(i)
	print(info)

linkstate = p.getLinkState(human, 25)
print("The link state or link 25 is: ", linkstate)
print("\n")

jointinfo = p.getJointInfo(human, 25)
print("The joint info or link 25 is: ", jointinfo)

bodies = p.getNumBodies()
print("The number of bodies is: ", bodies)	    # So far there are 2 bodies: 0 = plane, 1 = human
"""for i in range(bodies):
	info = p.getBodyInfo(i)
	body_id = p.getBodyUniqueId(i)
	print("Info: ",info)
	print("This body's Id is: ", body_id)"""


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
p.setRealTimeSimulation(1)
color = [1, 0, 0]

"""Joint indexes:
0: Waist
1: Between Feet (in floor)
4: Upper Chest
7: Neck
8: Head
25: left_wrist_2 (out of 3) """

forceVector = [0, 0, 1]
p.addUserDebugLine([0,0,0],
		   lineToXYZ=[1,1,1],
                   lineColorRGB= [1,0,0],
                   lineWidth=3,
		   parentObjectUniqueId=human,
		   parentLinkIndex = 25)
"""
p.applyExternalForce(objectUniqueId=1,
		     linkIndex = 25,
		     forceObj = forceVector,
		     posObj=[0,0,0],
		     flags=2)"""
"""
p.addUserDebugLine(lineFromXYZ=[0,0.8,1.35],
		   lineToXYZ=[0,0.8,2],
                   lineColorRGB= [1,0,0],
                   lineWidth=3)"""

"""
p.createConstraint(human,
		   22,
		   25,
		   25,
		   4,
		   [0,0,1],
		   [0,0,0],
		   [0,0,0])"""


p.applyExternalForce(objectUniqueId=1,
		     linkIndex = 25,
		     forceObj = [100,100,100],
		     posObj=[0,0,0],
		     flags=2)

# p.setJointMotorControlArray(human, range(6), p.POSITION_CONTROL, targetPositions=[0.1]*6)

for _ in range(100000000):
	p.stepSimulation()

