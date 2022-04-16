"""Handles scaling and renders assets.
License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

# 3rd party modules
import pygame

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

        self.assets = []

        pygame.display.init()
        pygame.display.set_caption(name)
        self._screen = pygame.display.set_mode(
            (self.window_width, self.window_length))

    def reset_assets(self):
        """Reset stored assets."""
        self.assets = []

    def add_asset(self, asset):
        """Store asset for rendering."""
        self.assets.append(asset)

    def transform_pos(self, x, y):
        """Transform from field coordinates to screen coordinates."""
        x = self.window_length - \
            (int(x * self.scaling) + (self.window_length // 2))
        y = self.window_width - \
            (int(y * self.scaling) + (self.window_width // 2))
        return y, x

    def scale_size(self, s):
        """Transform from meters to pixels."""
        return s * self.scaling

    def show(self):
        """Render all stored assets."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                raise self.ShutdownError()

        self._screen.fill(self.BACKGROUND_COLOR)
        for asset in self.assets:
            asset.blit(self._screen)

        pygame.display.flip()
