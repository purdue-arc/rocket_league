#!/usr/bin/env python3

"""
Deep learning interface for ROS.
"""

from rocket_league_drl import RocketLeagueInterface
from stable_baselines3 import PPO
import time, rospy

env = RocketLeagueInterface()
model = PPO("MlpPolicy", env, verbose=1)

steps = rospy.get_param('~training_steps', 5000)
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
