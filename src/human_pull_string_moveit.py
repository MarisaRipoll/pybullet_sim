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


# NUMBER OF JOINTS = 43 (However most of them are visualized at the base... strange)
joints = p.getNumJoints(human)
print("The number of Joints is: ", joints)

for i in range(joints):
	info = p.getJointInfo(human, i)
	print(info)
info = p.getJointInfo(human, 25)
print(info)

# NUMBER OF CONSTRAINTS = 0 (Unless I uncomments createConstraint function)
cons = p.getNumConstraints()
print("The number of Constraints is: ", cons)

for i in range(cons):
	info = p.getConstraintInfo(i)
	print(info)

# GET LINK STATES, JOINT STATES
linkstate = p.getLinkState(human, 25)
print("The link state or link 25 is: ", linkstate)
print("\n")

jointinfo = p.getJointInfo(human, 25)
print("The joint info or link 25 is: ", jointinfo)

# NUMBER OF BODIES = 2 (0 = plane, 1 = human)
bodies = p.getNumBodies()
print("The number of bodies is: ", bodies)
"""for i in range(bodies):
	info = p.getBodyInfo(i)
	body_id = p.getBodyUniqueId(i)
	print("Info: ",info)
	print("This body's Id is: ", body_id)"""


# COMMON SETUP
p.setGravity(0, 0, -9.81)
p.setTimeStep(-0.0001)
p.setRealTimeSimulation(1)
color = [1, 0, 0]




# IMPORTANT JOINT INDICES:

"""
00: Waist
01: Base - between the feet (on the floor)
04: Upper Chest
07: Neck
08: Head
12: Right Shoulder
13: Right Elbow
16: Right Wrist
21: Left Shoulder
22: Left Elbow
25: Left Wrist
30: Right Leg (Point where it joins the waist)
31: Right Knee
33: Right Foot
38: Left Leh (Point where it joins the waist)
39: Left Knee
41: Left Foot
"""




#################################################################################
#########                                                         ###############
#########     ALONA: THE QUESTIONS ARE ABOUT THE CODE BELOW :)    ###############
#########                                                         ###############
#################################################################################



# THIS DEBUGLINE IS ATTACHED TO THE WRIST BUT POINTS ARE DEFINED IN LINK FRAME

p.addUserDebugLine([0,0,0],				# starting point = wrist link frame origin
		   lineToXYZ=[1,1,1],			# ending point : how to set it to world frame?
                   lineColorRGB= [1,0,0],
                   lineWidth=3,
		   parentObjectUniqueId=human,
		   parentLinkIndex = 25)


# THE USERDEBUGLINE BELOW HAS BOTH STARTING AND ENDING POINTS FIXED IN THE WORLD FRAME
"""
p.addUserDebugLine(lineFromXYZ=[0,0.8,1.35],
		   lineToXYZ=[0,0.8,2],
                   lineColorRGB= [1,0,0],
                   lineWidth=3)"""

# IF I UNCOMMENT THE CONSTRAINT (CREATE IT), THEN THE FIGURE LOSES CONTROL
"""
p.createConstraint(human,		# ParentBodyUniqueId
		   22,			# ParentLinkIndex
		   25,			# childBodyUniqueId
		   25,			# childLinkIndex
		   4,			# Join Type: JOINT_GEAR
		   [0,0,1],		# Joint Axis
		   [0,0,0],		# ParentFramePosition
		   [0,0,0])		# childFramePosition"""		

# THIS IS THE BIGGEST ISSUE, THE FIGURE DOES NOT REACT TO THE FUNCTION APPLYEXTERNALFORCE

p.applyExternalForce(objectUniqueId=1,
		     linkIndex = 25,
		     forceObj = [100,100,100],
		     posObj=[0,0,0],
		     flags=2)		# 2 Flags: WORLD_FRAME and LINK_FRAME

# THE CODE LINE BELOW RECEIVES THE SAME REACTION AS THE CREATECONSTRAINT FUNCTION:
# p.setJointMotorControlArray(human, (21,22,25), p.POSITION_CONTROL)

for _ in range(100000000):
	p.stepSimulation()

