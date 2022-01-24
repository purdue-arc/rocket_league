#!/usr/bin/env python3
"""Real-time evaluation of the agent trained for the Rocket League project.
License:
  BSD 3-Clause License
  Copyright (c) 2022, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

from rktl_autonomy import RocketLeagueInterface
from stable_baselines3 import PPO
from os.path import expanduser
import rospy

# create interface
env = RocketLeagueInterface()

# load the model
weights = expanduser(rospy.get_param('~weights'))
model = PPO.load(weights)

# evaluate in real-time
env.registercallback(model.predict)
obs = env.reset()
while not rospy.is_shutdown():
    action, __ = model.predict(obs)
    obs, __, __, __ = env.step(action)
