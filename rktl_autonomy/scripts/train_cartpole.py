#!/usr/bin/env python3
"""Training script for the CartPoleInterface for testing.
License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

from rktl_autonomy import CartpoleInterface
from stable_baselines3 import PPO
import gym, time

def show_progress(model, episodes=5):
   env = gym.make("CartPole-v0")
   obs = env.reset()
   episodes = 0
   while episodes < 5:
      action, __ = model.predict(obs, deterministic=True)
      obs, __, done, __ = env.step(action)
      env.render()
      time.sleep(0.01)
      if done:
         obs = env.reset()
         episodes += 1
   env.close()

env = CartpoleInterface()
model = PPO("MlpPolicy", env, verbose=1)

print("showing untrained performance")
show_progress(model)

print("training on 10k steps")
model.learn(total_timesteps=10000)

print("showing trained performance")
show_progress(model)
