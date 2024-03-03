"""Contains the Window class.
License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

# 3rd party modules
import pygame

# Local modules
from visualizer.asset import Image, Rectangle, Lines, Circle

class Window(object):
    """Interfaces PyGame for rendering."""

    BACKGROUND_COLOR = (0, 0, 0)
    GOAL_COLOR = (0, 200, 0)
    WALL_COLOR = (0, 0, 0)

    class ShutdownError(Exception):
        """Exception for when pygame is shut down"""
        pass

    def __init__(self, map_width, map_length, wall_thickness, name='Rocket League Visualizer'):
        # Support for 1080p monitors
        self.scaling = 1080 * 0.5 / (map_length + (wall_thickness*2.))

        self.window_length = int(
            (map_length + (wall_thickness*2.)) * self.scaling)
        self.window_width = int(
            (map_width + (wall_thickness*2.)) * self.scaling)

        self.assets = {}

        pygame.display.init()
        pygame.display.set_caption(name)
        self._screen = pygame.display.set_mode(
            (self.window_width, self.window_length))

    def createAsset(self, id, width, length, initPos=None, imgPath=None, color=None, radius=None, lines=False, circle=False):
        width = int(width * self.scaling)
        length = int(length * self.scaling)

        if lines:
            self.assets[id] = Lines(color)
        elif circle:
            radius = int(radius * self.scaling)
            self.assets[id] = Circle(color, radius)
        elif imgPath is None:
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

    def updateAssetRadius(self, id, radius):
        radius = int(radius * self.scaling)
        self.assets[id].setRadius(radius)

    def updateAssetAngle(self, id, angle):
        self.assets[id].setAngle(angle)
    
    def resetAssetLines(self, id):
        self.assets[id].resetPoints()

    def show(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                raise self.ShutdownError()

        self._screen.fill(self.BACKGROUND_COLOR)
        for asset in self.assets.values():
            asset.blit(self._screen)

        pygame.display.flip()
