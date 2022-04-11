#!/usr/bin/env python3
"""Evaluate a whole directory of models for the rocket league project
License:
  BSD 3-Clause License
  Copyright (c) 2022, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

from rktl_autonomy import RocketLeagueInterface
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy
from os.path import expanduser
from sys import argv
from glob import glob
import uuid

if __name__ == '__main__':      # this is required due to forking processes
    run_id = str(uuid.uuid4())  # ALL running environments must share this
    print(f"RUN ID: {run_id}")

    # to pass launch args, add to env_kwargs: 'launch_args': ['render:=false', 'plot_log:=true']
    env = make_vec_env(RocketLeagueInterface, env_kwargs={'run_id':run_id,
        'launch_args':['render:=false', 'plot_log:=false']},
        n_envs=24, vec_env_cls=SubprocVecEnv)

    assert len(argv) >= 2
    for model_run_id in argv[1:]:
        print(f"Evaluating training run: {model_run_id}")
        model_dir = expanduser(f'~/catkin_ws/data/rocket_league/{model_run_id}')

        with open(f'{model_dir}/eval_log.txt', 'w') as logfile:
            logfile.write(f"Training Episodes, mean reward, std dev reward\n")

            for weight in glob(f'{model_dir}/rl_model_*_steps.zip'):
                model = PPO.load(weight.replace('.zip', ''))
                mu, sigma = evaluate_policy(model, env, n_eval_episodes=48)

                episodes = weight.replace(f'{model_dir}/rl_model_', '').replace('_steps.zip', '')
                logfile.write(f"{episodes}\t{mu:.3f}\t{sigma:.3f}\n")
                logfile.flush()

    env.close() # this must be done to clean up other processes
