#!/usr/bin/env python3
"""Tests for rewards in rocket_league_interface.py. This node will own the
interface object and evaluate it's output.
License:
  BSD 3-Clause License
  Copyright (c) 2022, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

import unittest, rostest
import numpy as np
from rktl_autonomy import RocketLeagueInterface

class TestStep(unittest.TestCase):
    def test_all(self):
        env = RocketLeagueInterface()

        # case 0
        obs, reward, done, __ = env.step(env.action_space.sample())
        self.assertTrue(np.allclose(obs, [0, 0, 0, 1, 0, 0, 0, 0, 0]), 'observation vector is incorrect')
        self.assertAlmostEqual(reward, -2.125, 'reward is incorrect')
        self.assertFalse(done, 'done is incorrect')

        # case 1
        obs, reward, done, __ = env.step(env.action_space.sample())
        self.assertTrue(np.allclose(obs, [0, 0, 0, -1, 0, 1, 0, 0, 0]), 'observation vector is incorrect')
        self.assertAlmostEqual(reward, 998.775, 'reward is incorrect')
        self.assertTrue(done, 'done is incorrect')

        # case 2
        obs, reward, done, __ = env.step(env.action_space.sample())
        self.assertTrue(np.allclose(obs, [2.3, 0, 0, 0, 0, 0, 0, 0, 0]), 'observation vector is incorrect')
        self.assertAlmostEqual(reward, -7.654, 'reward is incorrect')
        self.assertFalse(done, 'done is incorrect')

        # case 3
        obs, reward, done, __ = env.step(env.action_space.sample())
        self.assertTrue(np.allclose(obs, [0, -1.65, 0, 1, 0, 0, 0, 0, 0]), 'observation vector is incorrect')
        self.assertAlmostEqual(reward, 994.50275, 'reward is incorrect')
        self.assertTrue(done, 'done is incorrect')

        self.assertTrue(False, 'this is a test')

if __name__ == '__main__':
    rostest.rosrun('rktl_autonomy', 'test_rktl_rewards', TestStep)