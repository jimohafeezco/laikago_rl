import gym
import laikago_bot
import pybullet as p
import pybullet_data
import os
from gym.utils import seeding
from stable_baselines.common import make_vec_env

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines import PPO2
from gym import spaces
import numpy as np
env = gym.make('laikago-v0')

# n_cpu = 4
# env.reset()
for _ in range(1000):

        # obs = env.reset()

        model = PPO2(MlpPolicy, env, verbose=1)
        model.learn(total_timesteps=20)


        env.render()
        env.reset()












# while True:
#     obs = env.reset()
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
#     env.render()

# for _ in range(10):
#     env.render()
#     # observation=env.reset()
#     obs = env.reset()
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
#     # env.reset()
        # observation, reward, done, info = env.step(env.action_space.sample()) # take a random action