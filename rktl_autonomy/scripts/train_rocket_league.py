#!/usr/bin/env python3
"""Training script for the Rocket League project.
License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

from rktl_autonomy import RocketLeagueInterface
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.logger import configure
from stable_baselines3.common.callbacks import CheckpointCallback
from os.path import expanduser
import uuid

if __name__ == '__main__':      # this is required due to forking processes
    run_id = str(uuid.uuid4())  # ALL running environments must share this
    print(f"RUN ID: {run_id}")

    # to pass launch args, add to env_kwargs: 'launch_args': ['render:=false', 'plot_log:=true']
    env = make_vec_env(RocketLeagueInterface, env_kwargs={'run_id':run_id},
            n_envs=24, vec_env_cls=SubprocVecEnv)

    model = PPO("MlpPolicy", env)

    # log training progress as CSV
    log_dir = expanduser(f'~/catkin_ws/data/rocket_league/{run_id}')
    logger = configure(log_dir, ["stdout", "csv", "log"])
    model.set_logger(logger)

    # log model weights
    freq = 20833 # save 20 times
    # freq = steps / (n_saves * n_envs)
    callback = CheckpointCallback(save_freq=freq, save_path=log_dir)

    # run training
    steps = 240000000 # 240M (10M sequential)
    print(f"training on {steps} steps")
    model.learn(total_timesteps=steps, callback=callback)

    # save final weights
    print("done training")
    model.save(log_dir + "/final_weights")
    env.close() # this must be done to clean up other processes
