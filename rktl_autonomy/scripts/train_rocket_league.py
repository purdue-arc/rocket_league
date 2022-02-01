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

if __name__ == '__main__':
    run_id = str(uuid.uuid4())

    # env = RocketLeagueInterface(run_id=run_id)
    env = make_vec_env(RocketLeagueInterface, env_kwargs={'run_id':run_id},
            n_envs=4, vec_env_cls=SubprocVecEnv)

    model = PPO("MlpPolicy", env)

    # log training progress as CSV
    log_dir = expanduser(f'~/catkin_ws/data/rocket_league/{run_id}')
    logger = configure(log_dir, ["stdout", "csv", "log"])
    model.set_logger(logger)

    # log model weights
    freq = 25000
    callback = CheckpointCallback(save_freq=freq, save_path=log_dir)

    # run training
    steps = 10000
    print(f"training on {steps} steps")
    model.learn(total_timesteps=steps, callback=callback)

    # save final weights
    print("done training")
    model.save(log_dir + "/final_weights")
    env.close()
