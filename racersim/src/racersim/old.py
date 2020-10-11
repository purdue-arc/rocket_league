#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Simple 2D simulator to emulate the rocket league field.
"""
import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)

from Box2D import *

import math

timestep = 1.0 / 60
vel_iters, pos_iters = 6, 2

screenHeight = 600
screenWidth = 600

actualWidth = 2
ppm = 500.0

u = 0.01

maxForwardSpeed = 1
maxBackwardSpeed = -1
maxDriveForce = 0.0001
maxLateralImpulse = 0.03

carInit = ((screenWidth/2.0)/ppm, (screenHeight/2.0)/ppm)
print(screenWidth/ppm)
print(carInit)
class Tire:
    def __init__(self, world, length, width):
        self.bodyDef = b2BodyDef()
        self.bodyDef.type = b2_dynamicBody
        self.body = world.CreateBody(self.bodyDef)

        self.shape = b2PolygonShape(box=(width/2, length/2))
        self.body.CreateFixture(shape=self.shape, density=0.01)

    def getForwardVelocity(self):
        normal = self.body.GetWorldVector((0,1))
        return b2Dot(normal, self.body.linearVelocity) * normal

    def getLateralVelocity(self):
        normal = self.body.GetWorldVector((1,0))
        return b2Dot(normal, self.body.linearVelocity) * normal

    def updateFriction(self):
        impulse = self.body.mass * -self.getLateralVelocity()
        if impulse.length > maxLateralImpulse:
            impulse *= maxLateralImpulse / impulse.length
        self.body.ApplyLinearImpulse(impulse, self.body.worldCenter, wake=True)
        self.body.ApplyAngularImpulse(0.08 * self.body.inertia * \
                                      -self.body.angularVelocity, wake=True)

        currForwardNormal = self.getForwardVelocity()
        currForwardSpeed = currForwardNormal.Normalize()
        dragForce = -0.00002 * currForwardSpeed
        self.body.ApplyForce(dragForce * currForwardNormal, \
                             self.body.worldCenter, wake=True)

    def updateDrive(self, control):
        desiredSpeed = 0
        if control[0] == 1:
            desiredSpeed = maxForwardSpeed
        elif control[0] == -1:
            desiredSpeed = maxBackwardSpeed
        else:
            return

        currForwardNormal = self.body.GetWorldVector((0,1))
        currSpeed = b2Dot(self.getForwardVelocity(), currForwardNormal)

        force = 0
        if desiredSpeed > currSpeed:
            force = maxDriveForce
        elif desiredSpeed < currSpeed:
            force = -maxDriveForce
        else:
            return

        self.body.ApplyForce(force * currForwardNormal, \
                             self.body.worldCenter, wake=True)

    def updateTurn(self, control):
        desiredTorque = 0
        if control[1] == 1:
            desiredTorque = 0.0015
        elif control[1] == -1:
            desiredTorque = -0.0015
        else:
            return

        self.body.ApplyTorque(desiredTorque, wake=True)

    def draw(self, screen):
        for fixture in self.body.fixtures:
            shape = fixture.shape
            vertices = [self.body.transform * v * ppm for v in shape.vertices]
            vertices = [(v[0], v[1]) for v in vertices]
            pygame.draw.polygon(screen, (255, 255, 255), vertices)

class Car:
    def __init__(self, world):
        self.bodyDef = b2BodyDef()
        self.bodyDef.type = b2_dynamicBody
        self.bodyDef.position = carInit
        self.body = world.CreateBody(self.bodyDef)

        vertices = [(0.0225, 0), (0.0225, 0.155),
                    (-0.0225, 0.155), (-0.0225, 0)]
        self.shape = b2PolygonShape(vertices=vertices)
        self.fixture = self.body.CreateFixture(shape=self.shape, density=0.0124)

        self.tires = []

        jointDef = b2RevoluteJointDef()
        jointDef.bodyA = self.body
        jointDef.enableLimit = True
        jointDef.lowerAngle = 0
        jointDef.upperAngle = 0
        jointDef.localAnchorB.SetZero()

        tireLength = 0.037
        tireWidth = 0.0125

        tireFL = Tire(world, tireLength, tireWidth)
        jointDef.bodyB = tireFL.body
        jointDef.localAnchorA.Set(-0.03525, 0.135)
        self.flJoint = world.CreateJoint(jointDef)
        self.tires.append(tireFL)

        tireFR = Tire(world, tireLength, tireWidth)
        jointDef.bodyB = tireFR.body
        jointDef.localAnchorA.Set(0.03525, 0.135)
        self.frJoint = world.CreateJoint(jointDef)
        self.tires.append(tireFR)

        tireBL = Tire(world, tireLength, tireWidth)
        jointDef.bodyB = tireBL.body
        jointDef.localAnchorA.Set(-0.03525, 0.0185)
        world.CreateJoint(jointDef)
        self.tires.append(tireBL)

        tireBR = Tire(world, tireLength, tireWidth)
        jointDef.bodyB = tireBR.body
        jointDef.localAnchorA.Set(0.03525, 0.0185)
        world.CreateJoint(jointDef)
        self.tires.append(tireBR)

    def update(self, control):
        for tire in self.tires:
            tire.updateFriction()
        for tire in self.tires:
            tire.updateDrive(control)

        lockAngle = math.radians(30)
        turnSpeed = math.radians(320)
        turnPerTimeStep = turnSpeed / 60
        desiredAngle = 0

        if control[1] == 1:
            desiredAngle = lockAngle
        elif control[1] == -1:
            desiredAngle = -lockAngle

        angleNow = self.flJoint.angle
        angleToTurn = desiredAngle - angleNow
        if angleToTurn != 0:
            angleToTurn = math.copysign(turnPerTimeStep, angleToTurn)
        newAngle = angleNow + angleToTurn
        self.flJoint.SetLimits(newAngle, newAngle)
        self.frJoint.SetLimits(newAngle, newAngle)

    def draw(self, screen):
        for fixture in self.body.fixtures:
            shape = fixture.shape
            vertices = [self.body.transform * v * ppm for v in shape.vertices]
            vertices = [(v[0], v[1]) for v in vertices]
            pygame.draw.polygon(screen, (0, 25, 156), vertices)

        for tire in self.tires:
            tire.draw(screen)

class Ball():
    def __init__(self, world):
        self.bodyDef = b2BodyDef()
        self.bodyDef.type = b2_dynamicBody
        self.bodyDef.position = (screenWidth / ppm / 3, screenHeight / ppm / 3)
        self.body = world.CreateBody(self.bodyDef)

        self.shape = b2CircleShape(radius=(0.05))
        self.body.CreateFixture(shape=self.shape, density=0.003, restitution=0.8)

    def draw(self, screen):
        x,y = self.body.position
        x = int(x * ppm)
        y = int(y * ppm)
        pygame.draw.circle(screen, (255, 255, 255), (x, y), int(self.shape.radius * ppm))

def draw(body):
        for fixture in body.fixtures:
            shape = fixture.shape
            vertices = [body.transform * v for v in shape.vertices]
            vertices = [(v[0] * ppm, v[1] * ppm) for v in vertices]
            pygame.draw.polygon(screen, (100, 25, 125), vertices)


if __name__ == "__main__":
    pygame.init()
    screen = pygame.display.set_mode((screenHeight, screenWidth))
    clock = pygame.time.Clock()

    running = True

    world = b2World(gravity=(0,0))

    groundBody = world.CreateBody()
    groundShape = b2PolygonShape(box=(screenHeight/ppm, screenWidth/ppm))
    groundFixtureDef = b2FixtureDef(shape=groundShape)
    groundFixtureDef.isSensor = True
    groundFixture = groundBody.CreateFixture(groundFixtureDef, restitution=0.75)

    wallPositions = [(0,(screenHeight/ppm)),
                     ((screenWidth/ppm),0),
                     (0,0),
                     (0,0)]

    wallBodies = []
    for pos in range(len(wallPositions)):
        wallBody = world.CreateBody(position=wallPositions[pos])
        wallBodies.append(wallBody)
        if (pos % 2) == 0:
            wallShape = b2PolygonShape(box=(screenWidth/ppm, 0.02))
        else:
            wallShape = b2PolygonShape(box=(0.02, screenHeight/ppm))
        wallFixtureDef = b2FixtureDef(shape=wallShape)
        wallFixture = wallBody.CreateFixture(wallFixtureDef)

    car = Car(world)
    ball = Ball(world)

    forwardCommand = 0
    turnCommand = 0

    while running:
        screen.fill((0,0,0))
        events = pygame.event.get()

        for e in events:
            if e.type == pygame.QUIT:
                running = False
                break
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_UP:
                    forwardCommand = 1
                elif e.key == pygame.K_DOWN:
                    forwardCommand = -1
                elif e.key == pygame.K_RIGHT:
                    turnCommand = 1
                elif e.key == pygame.K_LEFT:
                    turnCommand = -1
            elif e.type == pygame.KEYUP:
                if e.key == pygame.K_UP or \
                    e.key == pygame.K_DOWN:
                    forwardCommand = 0
                elif e.key == pygame.K_RIGHT or \
                      e.key == pygame.K_LEFT:
                    turnCommand = 0

        car.draw(screen)
        ball.draw(screen)
        for wall in wallBodies:
            draw(wall)
        world.Step(timestep, vel_iters, pos_iters)
        car.update((forwardCommand, turnCommand))
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
