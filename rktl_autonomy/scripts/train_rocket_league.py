#!/usr/bin/env python3
"""Training script for the Rocket League project.
License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

from rktl_autonomy import RocketLeagueInterface
#import numpy as np
from stable_baselines3 import PPO
import torch
import torch.nn as nn
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
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
<<<<<<< HEAD
    env = make_vec_env(RocketLeagueInterface, env_kwargs={'run_id':run_id},
            n_envs=24, vec_env_cls=SubprocVecEnv)
=======
    env = make_vec_env(RocketLeagueInterface, env_kwargs={'run_id':run_id}, n_envs=24, vec_env_cls=SubprocVecEnv)
>>>>>>> dff0f278ab7adc1e9262b61607da888895e44405

    model = PPO("MlpPolicy", env, n_steps= 163, gamma= 0.9973997734112163, learning_rate= 0.000875486766488368, ent_coef= 4.805419210954314e-06, clip_range= 0.1, n_epochs= 16, gae_lambda= 0.9614821689691063, batch_size= 3177, vf_coef= 0.7501914642632537)

    
    # log training progress as CSV
    log_dir = expanduser(f'~/catkin_ws/data/rocket_league/{run_id}')
    logger = configure(log_dir, ["stdout", "csv", "log"])
    model.set_logger(logger)

    # log model weights
<<<<<<< HEAD
    freq = 20833 # save 20 times
    # freq = steps / (n_saves * n_envs)
    callback = CheckpointCallback(save_freq=freq, save_path=log_dir)

    # run training
    steps = 240000000 # 240M (10M sequential)
=======
    freq = 5000 # (time steps in a SINGLE environment)
    # ex. To save 20 times with 10M timesteps on 10 vec_envs, set to 50k
    callback = CheckpointCallback(save_freq=freq, save_path=log_dir)

    # run training
    steps = 100000000 # 10M (timesteps accross ALL environments)
>>>>>>> dff0f278ab7adc1e9262b61607da888895e44405
    print(f"training on {steps} steps")
    model.learn(total_timesteps=steps, callback=callback)

    # save final weights
    print("done training")
    model.save(log_dir + "/final_weights")
    env.close() # this must be done to clean up other processes
