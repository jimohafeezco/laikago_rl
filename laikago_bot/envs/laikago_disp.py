import pybullet as p
import time
import gym
from gym import spaces
from gym.utils import seeding

import os
import pybullet as p
import pybullet_data

import time
import math
import numpy as np
p.connect(p.GUI)
# p.setGravity(0,0,-9.8)
p.setTimeStep(1./500)
urdfFlags = p.URDF_USE_SELF_COLLISION
quadruped=p.loadURDF("laikago/laikago.xml",[0,0,.5],[0,0.5,0.5,0], flags = urdfFlags,useFixedBase=False)

num_joint=p.getNumJoints(quadruped)
action_dim = 8
action_bound=1
action_high = np.array([action_bound] * action_dim)
action_space1 = spaces.Discrete(9)
action_space = spaces.Box(-action_high, action_high)		
print(action_space)
print(action_space1)
print(num_joint)
while (2>1):
  p.stepSimulation()
