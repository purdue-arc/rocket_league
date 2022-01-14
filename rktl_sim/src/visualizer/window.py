#!/usr/bin/env python3

"""Contains the Window class.

License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

# 3rd party modules
import math
import pygame

# Local modules
from visualizer.asset import Asset, Image, Rectangle


class Window(object):
    """Interfaces PyGame for rendering."""

    BACKGROUND_COLOR = (200, 200, 200)
    GOAL_COLOR = (0, 200, 0)
    WALL_COLOR = (0, 0, 0)

    class ShutdownError(Exception):
        """Exception for when pygame is shut down"""
        pass

    def __init__(self, map_width, map_length, wall_thickness):
        # Support for 1080p monitors
        self.scaling = 1080 * 0.7 / (map_length + wall_thickness)

        self.window_length = int(
            (map_length + wall_thickness) * self.scaling)
        self.window_width = int(
            (map_width + wall_thickness) * self.scaling)

        self.assets = {}

        pygame.display.init()
        self._screen = pygame.display.set_mode(
            (self.window_width, self.window_length))

    def createAsset(self, id, width, length, initPos=None, imgPath=None, color=None):
        width = int(width * self.scaling)
        length = int(length * self.scaling)

        if imgPath is None:
            self.assets[id] = Rectangle(width, length, color)
        else:
            self.assets[id] = Image(width, length, imgPath)

        if initPos is not None:
            self.updateAssetPos(id, initPos[0], initPos[1])

    def updateAssetPos(self, id, x, y):
        # Adjust for simulation coordinate frame
        x = self.window_length - \
            (int(x * self.scaling) + (self.window_length // 2))
        y = self.window_width - \
            (int(y * self.scaling) + (self.window_width // 2))
        self.assets[id].setPos(y, x)

    def updateAssetAngle(self, id, angle):
        self.assets[id].setAngle(angle)

    def show(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                raise self.ShutdownError()

        self._screen.fill(self.BACKGROUND_COLOR)
        for asset in self.assets.values():
            asset.blit(self._screen)

        pygame.display.flip()