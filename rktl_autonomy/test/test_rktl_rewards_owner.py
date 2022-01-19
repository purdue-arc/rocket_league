#!/usr/bin/env python3
"""Tests for rewards in rocket_league_interface.py. This node will own the
interface object and evaluate it's output.
License:
  BSD 3-Clause License
  Copyright (c) 2022, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

import unittest, rostest
from rktl_autonomy import RocketLeagueInterface

class TestStep(unittest.TestCase):
    def test_all(self):
        env = RocketLeagueInterface()
        obs, reward, done, __ = env.step(env.action_space.sample())
        obs, reward, done, __ = env.step(env.action_space.sample())
        obs, reward, done, __ = env.step(env.action_space.sample())
        obs, reward, done, __ = env.step(env.action_space.sample())

if __name__ == '__main__':
    rostest.rosrun('rktl_autonomy', 'test_rktl_rewards', TestStep)