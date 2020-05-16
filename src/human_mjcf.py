import argparse
import os

import pybullet as p
import pybullet_data
import math
import time

p.connect(p.GUI)
p.resetSimulation()

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
p.setPhysicsEngineParameter(numSolverIterations=5)
p.setPhysicsEngineParameter(fixedTimeStep=1. / 240.)
p.setPhysicsEngineParameter(numSubSteps=1)

plane = p.loadURDF("plane.urdf")

human = p.loadMJCF("mjcf/humanoid_symmetric.xml", 0)

ob = human[0]
p.resetBasePositionAndOrientation(ob, [0, 0, 1.15],
                                  [0, 0, 0, 1])
jointPositions = [
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0
]
for jointIndex in range(p.getNumJoints(ob)):
  p.resetJointState(ob, jointIndex, jointPositions[jointIndex])


p.setGravity(0, 0, -9.81)
p.setTimeStep(-0.0001)
p.setRealTimeSimulation(0)

# p.setJointMotorControlArray(human, range(6), p.POSITION_CONTROL, targetPositions=[0.1]*6)

for _ in range(100000):
	time.sleep(5)
	p.stepSimulation()

