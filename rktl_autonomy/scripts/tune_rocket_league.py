#!/usr/bin/env python3
"""Training script for the Rocket League project.
License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

from rktl_autonomy import RocketLeagueInterface
import numpy as np
from stable_baselines3 import PPO
import torch
import torch.nn as nn
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
#from stable_baselines3.common.cmd_util import make_vec_env
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.logger import configure
from stable_baselines3.common.callbacks import CheckpointCallback
from os.path import expanduser
import uuid
import optuna

run_id = str(uuid.uuid4())  # ALL running environments must share this
step = 0

def optimize_ppo2(trial):
    """ Learning hyperparamters we want to optimise"""
    return {
        'n_steps': int(trial.suggest_loguniform('n_steps', 16, 2048)),
        'gamma': trial.suggest_loguniform('gamma', 0.9, 0.9999),
        'learning_rate': trial.suggest_loguniform('learning_rate', 1e-5, 1.),
        'ent_coef': trial.suggest_loguniform('ent_coef', 1e-8, 1e-1),
        'clip_range': trial.suggest_uniform('clip_range', 0.1, 0.4),
        # 'noptepochs': int(trial.suggest_loguniform('noptepochs', 1, 48)),
        'gae_lambda': trial.suggest_uniform('gae_lambda', 0.8, 1.)
    }


def optimize_agent(trial):
	
    global step
    
    env_count = 1 # Number of environments to run in parallel (keep at 1 for now)
    timesteps = 100 # Timesteps that the model will train on before getting assessed and tuned
    episodes_per_trial = 1 # Number of episode attempts (runs) per trial
    
    ### Train the model and optimize it.
    
    # Hyperparams are determined by Optuna.
    model_params = optimize_ppo2(trial)
    
    env = make_vec_env(RocketLeagueInterface, env_kwargs={'run_id' : run_id, 'launch_args': ['render:=false', 'plot_log:=false']},
            n_envs= env_count , vec_env_cls=SubprocVecEnv) # Creates the env
    
    model = PPO("MlpPolicy", env, **model_params) # Creates the PPO network model based on the env and the provided hyperparameters.
    
    model.learn(timesteps) # Makes the model learn before getting assessed and tuned
    
    rewards = [] # The way the car was rewarded for each attempt it had in this tuning trial
    
    n_episodes, reward_sum = 0, 0.0

    obs = env.reset()
    while n_episodes < episodes_per_trial:
        action, _ = model.predict(obs)
        
        if not obs.all() == obs.all():
        	print("\nOBS BAD\n")
        
        obs, reward, done, _ = env.step(action)
        
        reward_sum += reward

        if done:
            rewards.append(reward_sum)
            reward_sum = 0.0
            n_episodes += 1
            obs = env.reset()

    mean_reward = np.mean(rewards)
    
    step += 1
    
    print("\n")
    print("Mean Reward at step #" + str(step) + " : " + str(mean_reward) )

    env.close() # this must be done to clean up other processes
    
    # Optuna maximizes negative reward, so we need to negate the reward here
    trial.report(-1 * mean_reward, step)
    
    return -1 * mean_reward
    
    



if __name__ == '__main__':      # this is required due to forking processes
    
    study = optuna.create_study()
    
    try:
        study.optimize(optimize_agent, n_trials=10000, n_jobs=24)
        print("\n")
        print("Best Params:" + str(study.best_params))
    except KeyboardInterrupt:
        print('Interrupted by keyboard.')
        print("\n")
        print("Best Params:" + str(study.best_params))
    
    
    

    

