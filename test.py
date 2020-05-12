import gym
import laikago_bot
import pybullet as p
import pybullet_data
import os
from gym.utils import seeding

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines import PPO2
from gym import spaces
import numpy as np
env = gym.make('laikago-v0')
# print(observation)
for _ in range(1000):
    env.render()
    observation=env.reset()

    observation, reward, done, info = env.step(env.action_space.sample()) # take a random action
