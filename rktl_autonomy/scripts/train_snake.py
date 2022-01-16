#!/usr/bin/env python3
"""Training script for the Snake game in ARC tutorials.
License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

from rktl_autonomy import SnakeInterface
from stable_baselines3 import PPO
from stable_baselines3.common.logger import configure
from stable_baselines3.common.callbacks import CheckpointCallback
from os.path import expanduser, normpath
import rospy

env = SnakeInterface()
model = PPO("MlpPolicy", env, verbose=1)

# log training progress as CSV
log_dir = normpath(expanduser(rospy.get_param('~log/base_dir'))) + "/" + env.get_run_uuid()
logger = configure(log_dir, ["stdout", "csv"])
model.set_logger(logger)

# log model weights
freq = rospy.get_param('~log/model_freq', 2048)
callback = CheckpointCallback(save_freq=freq, save_path=log_dir)

# run training
steps = rospy.get_param('~training_steps', 50000)
print(f"training on {steps} steps")
model.learn(total_timesteps=steps, callback=callback)

# save final weights
print("done training")
model.save(log_dir + "/final_weights")
