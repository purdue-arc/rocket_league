"""
This package manipulates a ROS network so that it provides an enviornment to
train an AI. In short, it uses the sim time mechanism used to replay bag files
to mess with how time progresses, so that an arbitrary ROS network can be used
as an environment, specifically, an OpenAI Gym environment.

This can be used for both training and evaluation purposes. When training, it
manipulates time so that the network gets what it needs. It evaluation, it
runs the network as fast as real time is progressing.

License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

from ._ros_interface import ROSInterface
from .cartpole_interface import CartpoleInterface
from .cartpole_direct_interface import CartpoleDirectInterface
from .snake_interface import SnakeInterface
from .rocket_league_interface import RocketLeagueInterface
from .env_counter import EnvCounter

__all__ = [
    "ROSInterface",
    "CartpoleInterface",
    "CartpoleDirectInterface",
    "SnakeInterface",
    "RocketLeagueInterface",
    "EnvCounter"]