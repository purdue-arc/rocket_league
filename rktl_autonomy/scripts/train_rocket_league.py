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

    # to pass launch args, add to env_kwargs: 'launch_args': ['render:=false', 'plot_log:=true']
    env = make_vec_env(RocketLeagueInterface, env_kwargs={'run_id':run_id},
            n_envs=2, vec_env_cls=SubprocVecEnv)
            
    # The noise objects for DDPG
    #n_actions = env.action_space.shape[-1]
    #action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

    #model = DDPG("MlpPolicy", env, action_noise=action_noise)

    model = PPO("MlpPolicy", env, batch_size= 64, n_steps= 512, gamma= 0.995, learning_rate= 1e-04, ent_coef= 0.001, clip_range= 0.3, gae_lambda= 0.9, max_grad_norm= 0.7, vf_coef= 0.430793, policy_kwargs= dict(log_std_init=-2, ortho_init=False, activation_fn=nn.ReLU, net_arch=[dict(pi=[512, 512], vf=[512, 512])]))
    
    # log training progress as CSV
    log_dir = expanduser(f'~/catkin_ws/data/rocket_league/{run_id}')
    logger = configure(log_dir, ["stdout", "csv", "log"])
    model.set_logger(logger)

    # log model weights
    freq = 12500 # (time steps in a SINGLE environment)
    # ex. To save 20 times with 10M timesteps on 10 vec_envs, set to 50k
    callback = CheckpointCallback(save_freq=freq, save_path=log_dir)

    # run training
    steps = 100000000 # 10M (timesteps accross ALL environments)
    print(f"training on {steps} steps")
    model.learn(total_timesteps=steps, callback=callback)

    # save final weights
    print("done training")
    model.save(log_dir + "/final_weights")
    env.close() # this must be done to clean up other processes
