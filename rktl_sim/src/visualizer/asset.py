"""Contains the Asset class.
License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

# 3rd Party Modules
import math
import pygame
from abc import ABC, abstractmethod


class Asset(ABC):
    @abstractmethod
    def setPos(self):
        pass

    @abstractmethod
    def blit(self):
        pass


class Image(Asset):
    def __init__(self, width, length, img_path):
        self.width = width
        self.length = length
        self.pos = (0, 0)
        self.angle = 0

        self.init_img = pygame.image.load(img_path)
        self.init_img = pygame.transform.scale(
            self.init_img, (width, length))
        self.img = self.init_img

    def setPos(self, x, y):
        self.pos = (x, y)

    def setAngle(self, angle):
        if angle < 0:
            angle = 360. + angle

        self.img = pygame.transform.rotate(self.init_img, angle)

    def blit(self, screen):
        rect = self.img.get_rect(center=self.pos)
        screen.blit(self.img, rect)


class Rectangle(Asset):
    def __init__(self, width, length, color):
        self.color = color
        self.rect = pygame.Rect(0, 0, width, length)

    def setPos(self, x, y):
        self.pos = (x, y)
        self.rect.center = self.pos

    def blit(self, screen):
        pygame.draw.rect(screen, self.color, self.rect)

class Lines(Asset):
    def __init__(self, color):
        self.color = color
        self.points = []
    
    def resetPoints(self):
        self.points = []

    def setPos(self, x, y):
        self.points.append((x, y))

    def blit(self, screen):
        if len(self.points) > 1:
            pygame.draw.lines(screen, self.color, False, self.points)

class Circle(Asset):
    def __init__(self, color, radius):
        self.color = color
        self.radius = radius
        self.pos = (0, 0)

    def setPos(self, x, y):
        self.pos = (x, y)
    
    def setRadius(self, radius):
        self.radius = radius

    def blit(self, screen):
        pygame.draw.circle(screen, self.color, self.pos, self.radius, 5)
