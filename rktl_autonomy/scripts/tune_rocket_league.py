#!/usr/bin/env python3
"""Training script for the Rocket League project.
License:
  BSD 3-Clause License
  Copyright (c) 2022, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

from rktl_autonomy import RocketLeagueInterface
import numpy as np
from stable_baselines3 import PPO

from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.env_util import make_vec_env

import uuid
import optuna
import sys

from stable_baselines3.common.vec_env import VecCheckNan



timesteps = 2000 # Timesteps that the model will train on before getting assessed and tuned
episodes_per_trial = 7 # Number of times the model will be evaluated per tuning trial after training.
total_trials = 10000 # Number of optimization attempts optuna will make to the hyperparams.
parallel_jobs = 24 # Number of parallel envs the script will run and use at once.


def optimize_ppo2(trial):
    """ Learning hyperparamters we want to optimize"""
    return {
        'n_steps': int(trial.suggest_loguniform('n_steps', 16, 2048)),
        'gamma': trial.suggest_loguniform('gamma', 0.9, 0.9997),
        'learning_rate': trial.suggest_loguniform('learning_rate', 5e-6, 0.003), 
        'ent_coef': trial.suggest_loguniform('ent_coef', 1e-8, 0.01), 
        'clip_range': trial.suggest_uniform('clip_range', 0.1, 0.3), # try 0.2 and 0.3 as well
        'n_epochs': int(trial.suggest_loguniform('n_epochs', 3, 30)),
        'gae_lambda': trial.suggest_uniform('gae_lambda', 0.9, 1.0),
        'batch_size': int(trial.suggest_uniform('batch_size', 4, 4096)),
        'vf_coef': trial.suggest_uniform('v_coef', 0.5, 1.0)
        #'max_grad_norm': trial.suggest_uniform('max_grad_norm', 0.9, 1.0)  
    }



### Global variables. Not for modification.
run_id = str(uuid.uuid4())  # ALL running environments must share this id
step = 0 # Keeps track of tuning steps, i.e. how many unique hyperparam sets it has run.
study = optuna.create_study() # Creates optuna study.

### Train the model and optimize it.
def optimize_agent(trial):
    
    global step
    global study
    
    # Hyperparams are determined by Optuna.
    model_params = optimize_ppo2(trial)
    
    env = make_vec_env(RocketLeagueInterface, env_kwargs={'run_id' : run_id, 'launch_args': ['render:=false', 'plot_log:=false']},
            n_envs= 1 , vec_env_cls=SubprocVecEnv) # Creates the env
            
    env = VecCheckNan(env, raise_exception=True)
    
    model = PPO("MlpPolicy", env, **model_params) # Creates the PPO network model based on the env and the provided hyperparameters.
    
    model.learn(timesteps) # Makes the model learn before getting assessed and tuned
    
    rewards = [] # The way the car was rewarded for each attempt it had in this tuning trial
    
    n_episodes, reward_sum = 0, 0.0

    obs = env.reset()
    while n_episodes < episodes_per_trial:
    
        if np.isnan(obs).any():
            print("Best Params:" + str(study.best_params))
            sys.exit("OBS is NAN")
  
        action, _ = model.predict(obs)
               
        if np.isnan(action).any():
            print("Best Params:" + str(study.best_params))
            sys.exit("ACTION IS NAN")
               
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
    if (step + 1) % 5 == 0 :
        print("Best Params:" + str(study.best_params))

    env.close() # this must be done to clean up other processes
    
    # Optuna maximizes negative reward, so we need to negate the reward here
    trial.report(-1 * mean_reward, step)
    
    return -1 * mean_reward
    
    


if __name__ == '__main__':      # this is required due to forking processes
    
    try: # Begins the optimizer with total trials and number of parallel jobs specified.
        study.optimize(optimize_agent, n_trials=total_trials, n_jobs=parallel_jobs)
        print("\n")
        print("Best Params:" + str(study.best_params))
    except KeyboardInterrupt:
        print('Interrupted by keyboard.')
        print("\n")
        print("Best Params:" + str(study.best_params))
    
    