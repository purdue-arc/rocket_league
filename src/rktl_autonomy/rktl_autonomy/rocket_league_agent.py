#!/usr/bin/env python3
"""Real-time evaluation of the agent trained for the Rocket League project.
License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

from rktl_autonomy.rocket_league_interface import RocketLeagueInterface
from stable_baselines3 import PPO
from os.path import expanduser
from rclpy.exceptions import ROSInterruptException


def main():
  # create interface (and init ROS)
  env = RocketLeagueInterface(eval=True)

  # load the model
  weights = expanduser(env.node.get_parameter('~weights').get_parameter_value().string_value)
  model = PPO.load(weights)

  # evaluate in real-time
  obs = env.reset()
  while True:
      action, __ = model.predict(obs)
      try:
          obs, __, __, __ = env.step(action)
      # except rospy.ROSInterruptException:
      except ROSInterruptException:
          exit()

if __name__=='__main__':
    main()
    