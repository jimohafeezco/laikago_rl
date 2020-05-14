

import gym
from baselines import deepq
import laikago_bot
from baselines.common import models
import pybullet as p





import gym

from baselines import deepq
from baselines.common import models


def main():
        env = gym.make("laikago-v0")
    # for i in range(20):
    # Enabling layer_norm here is import for parameter space noise!
    # for i in range(1000):
    # for i in range(1000):
        observation = env.reset()
        # print(observation)
        act = deepq.learn(
            env,
            network=models.mlp(num_hidden=64, num_layers=1),
            lr=1e-3,
            total_timesteps=100000,
            buffer_size=50000,
            exploration_fraction=0.1,
            exploration_final_eps=0.1,
            print_freq=10
        )



if __name__ == '__main__':
    main()
