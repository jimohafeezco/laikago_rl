import gym
from gym import spaces
from gym.utils import seeding

import os
import pybullet as p
import pybullet_data

import time
import math
import numpy as np


class WalkingLaikagoEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def __init__(self, render=True):
        super(WalkingLaikagoEnv, self).__init__()
        # self.action_space = spaces.Discrete(8)
        self._action_bound=1
        self._time_step = 0.01
        action_dim = 8
        # action_high = np.array([self._action_bound] * action_dim)
        self.action_space = spaces.Discrete(9)
        # self.action_space = spaces.Box(-action_high, action_high)
        self.observation_space = spaces.Box(
            low=-1, high=1, shape=(29,), dtype=np.float32)

        self._observation = []
        if (render):
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)  # non-graphical version
        p.setAdditionalSearchPath(
            pybullet_data.getDataPath())  # used by loadURDF
        p.resetDebugVisualizerCamera(
            cameraDistance=0.8, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
        self._seed()
        # self.reset()
# [0,0,.5],[0,0.5,0.5,0]
        p.resetSimulation()
        p.setGravity(0, 0, -10)  # m/s^2
        # p.setTimeStep(1./60.)   # sec
        p.setTimeStep(0.01)   # sec
        self.plane = p.loadURDF("plane.urdf")

        self.cubeStartPos = [0,0,.5]
        self.cubeStartOrientation =[0,0.5,0.5,0]
        path = os.path.abspath(os.path.dirname(__file__))
        self.robotId = p.loadURDF(
            os.path.join(path, "laikago/laikago.xml"),
            self.cubeStartPos,
            self.cubeStartOrientation
        )
        self.movingJoints = [0, 2, 3, 5, 6, 8, 9, 11]

    # def reset(self):
    #     p.setGravity(0,0,-9.8)
    #     p.setTimeStep(1)
    #     self.vt = [0, 0, 0, 0, 0, 0, 0, 0]
    #     self.vd = 0
    #     # self.maxV = 60 # 0.12sec/60 deg = 500 deg/s = 8.72 rad/s
    #     self.envStepCounter = 0
    #     p.resetBasePositionAndOrientation(
    #         self.robotId,
    #         posObj=self.cubeStartPos,
    #         ornObj=self.cubeStartOrientation
    #     )
    #     p.resetSimulation()
    #     p.setPhysicsEngineParameter(self._time_step)
    #     #   numSolverIterations=int(300))
    #     p.setTimeStep(self._time_step)
    #     path = os.path.abspath(os.path.dirname(__file__))

    #     self.robotId = p.loadURDF(
    #         os.path.join(path, "laikago/laikago.xml"),
    #         self.cubeStartPos,
    #         self.cubeStartOrientation
    #     )
    #     observation = self.compute_observation()
    #     return observation
    def reset(self):
        # reset is called once at initialization of simulation
        self.vt = 0
        self.vd = 0
        self.maxV = 24.6 # 235RPM = 24,609142453 rad/sec
        self._envStepCounter = 0
        self.envStepCounter =0
        p.resetSimulation()
        p.setGravity(0,0,-10) # m/s^2
        # p.setTimeStep(0.01) # sec
        planeId = p.loadURDF("plane.urdf")
        path = os.path.abspath(os.path.dirname(__file__))
        self.robotId = p.loadURDF(
            os.path.join(path, "laikago/laikago.xml"),
            self.cubeStartPos,
            self.cubeStartOrientation
        )
        # you *have* to compute and return the observation from reset()
        observation = self.compute_observation()
        return observation

    def step(self, action):
        self.assign_throttle(action)
        observation = self.compute_observation()
        reward = self.compute_reward()
        done = self.termination()
        self.envStepCounter += 1
        # print(self.envStepCounter)
        return observation, reward, done, {}
    

    def clamp(self, n, minn, maxn):
        return max(min(maxn, n), minn)
    
    def moveLeg(self, robot, id, target):
      if(robot is None):
          return
      p.setJointMotorControl2(
          bodyUniqueId=robot,
          jointIndex=id,
          controlMode=p.POSITION_CONTROL,  # controlMode   = p.VELOCITY_CONTROL,        #p.POSITION_CONTROL,
          targetPosition=target # targetVelocity=target # targetVelocity= target                     #targetPosition=position,
      )    

    def assign_throttle(self, action):
      for i, key in enumerate(self.movingJoints):
        # print(action)
        # self.vt = self.clamp(self.vt + action, -2, 2)
        # print(self.vt)
        # self.moveLeg(robot=self.robotId, id=key,  target=self.vt)

        dv = 0.1
        deltav = [-10.*dv,-5.*dv, -2.*dv, -0.1*dv, 0, 0.1*dv, 2.*dv,5.*dv, 10.*dv][action]
        vt = self.clamp(self.vt + deltav, -self.maxV, self.maxV)
        self.vt = vt
        self.moveLeg(robot=self.robotId, id=key,  target=self.vt)



    def compute_observation(self):
        observation=[]
        baseOri = np.array(p.getBasePositionAndOrientation(self.robotId))
        JointStates = p.getJointStates(self.robotId, self.movingJoints)
        BaseAngVel = p.getBaseVelocity(self.robotId)
        # ContactPoints = p.getContactPoints(self.robotId, self.plane)

        observation = np.array([
            baseOri[0][0],
            baseOri[0][1],
            baseOri[0][2],  # z (height) of the Torso -> 1
            # orientation (quarternion x,y,z,w) of the Torso -> 4
            baseOri[1][0],
            baseOri[1][1],
            baseOri[1][2],
            baseOri[1][3],
            JointStates[0][0],  # Joint angles(Pos) -> 8
            JointStates[1][0],
            JointStates[2][0],
            JointStates[3][0],
            JointStates[4][0],
            JointStates[5][0],
            JointStates[6][0],
            JointStates[7][0],
            # 3-dim directional velocity and 3-dim angular velocity -> 3+3=6
            BaseAngVel[0][0],
            BaseAngVel[0][1],
            BaseAngVel[0][2],
            BaseAngVel[1][0],
            BaseAngVel[1][1],
            BaseAngVel[1][2],
            JointStates[0][1],  # Joint Velocities -> 8
            JointStates[1][1],
            JointStates[2][1],
            JointStates[3][1],
            JointStates[4][1],
            JointStates[5][1],
            JointStates[6][1],
            JointStates[7][1]
        ])
        print(observation)
        return observation.tolist()

    def compute_reward(self):
        baseOri = np.array(p.getBasePositionAndOrientation(self.robotId))
        xposbefore = baseOri[0][0]

        BaseAngVel = p.getBaseVelocity(self.robotId)
        xvelbefore = BaseAngVel[0][0]

        p.stepSimulation()
        
        baseOri = np.array(p.getBasePositionAndOrientation(self.robotId))
        xposafter = baseOri[0][0]

        BaseAngVel = p.getBaseVelocity(self.robotId)
        xvelafter  = BaseAngVel[0][0]

        # forward_reward = (xposafter - xposbefore)
        forward_reward = 20 * (xvelbefore - xvelafter)


        JointStates = p.getJointStates(self.robotId, self.movingJoints)
        torques = np.array([np.array(joint[3]) for joint in JointStates])
        ctrl_cost = 1.0 * np.square(torques).sum()

        ContactPoints = p.getContactPoints(self.robotId, self.plane)
        contact_cost = 5 * 1e-1 * len(ContactPoints)
        # survive_reward = 1.0
        survive_reward = 0.0
        reward = forward_reward - ctrl_cost - contact_cost + survive_reward
        # print("Reward ", reward , "Contact Cost ", contact_cost, "forward reward ",forward_reward, "Control Cost ", ctrl_cost)
        # print("Reward ", reward)
        reward = reward if reward > 0 else 0

        p.addUserDebugLine(lineFromXYZ=(0, 0, 0), lineToXYZ=(
            0.3, 0, 0), lineWidth=5, lineColorRGB=[0, 255, 0], parentObjectUniqueId=self.robotId)
        p.addUserDebugText("Rewards {}".format(
            reward), [0, 0, 0.3],lifeTime=0.25, textSize=2.5, parentObjectUniqueId=self.robotId)
        #   print("Forward Reward", reward )
        #   print("Forward Reward", forward_reward, ctrl_cost, contact_cost , xvelbefore, xvelafter, xposbefore, xposafter )

        return reward 

    def compute_done(self):
        cubePos = np.array(p.getBasePositionAndOrientation(self.robotId))
        state=self.envStepCounter >= 100
        # print(self.envStepCounter)
        state1 = cubePos[0][2] < 0.45
        print(cubePos[0][2])
        # print(cubePos[2])
        return state1 or state
    #   return False or s
    def is_fallen(self):

        orientation = p.getBasePositionAndOrientation(self.robotId)[1]
        rot_mat = p.getMatrixFromQuaternion(orientation)
        local_up = rot_mat[6:]
        pos = p.getBasePositionAndOrientation(self.robotId)[0]
        #  or pos[2] < 0.13
        return (np.dot(np.asarray([0, 0, 1]), np.asarray(local_up)) < 0.85)

    def termination(self):
        position = p.getBasePositionAndOrientation(self.robotId)[0]
        # distance = math.sqrt(position[0] ** 2 + position[1] ** 2)
       
        state=self.envStepCounter >=100
        return position[2] < 0.45 or state
        #     print("LOW POSITION")
        # or position[2] <= 0.12
        # return self.is_fallen() or o[1] < -0.13 

    def render(self, mode='human', close=False):
      pass