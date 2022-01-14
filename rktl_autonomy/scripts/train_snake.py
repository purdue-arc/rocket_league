#!/usr/bin/env python3
"""Training script for the Snake game in ARC tutorials.
License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

from rktl_autonomy import SnakeInterface
from stable_baselines3 import PPO
import time, rospy

env = SnakeInterface()
model = PPO("MlpPolicy", env, verbose=1)

steps = rospy.get_param('~training_steps', 50000)
print(f"training on {steps} steps")
model.learn(total_timesteps=steps)

print("done training")
obs = env.reset()
while True:
   action, __ = model.predict(obs, deterministic=True)
   obs, __, done, __ = env.step(action)
   time.sleep(0.01)
   if done:
      obs = env.reset()
