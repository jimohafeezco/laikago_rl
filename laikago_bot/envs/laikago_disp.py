import pybullet as p
import time
p.connect(p.GUI)
# p.setGravity(0,0,-9.8)
p.setTimeStep(1./500)
urdfFlags = p.URDF_USE_SELF_COLLISION
quadruped=p.loadURDF("laikago/laikago.xml",[0,0,.5],[0,0.5,0.5,0], flags = urdfFlags,useFixedBase=False)

num_joint=p.getNumJoints(quadruped)
		
print(num_joint)
while (2>1):
  p.stepSimulation()
