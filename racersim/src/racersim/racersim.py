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

maxForwardSpeed = 100
maxBackwardSpeed = -20
maxDriveForce = 150
maxLateralImpulse = 3

class Tire:
    def __init__(self, world, length, width):
        self.bodyDef = b2BodyDef()
        self.bodyDef.type = b2_dynamicBody
        self.body = world.CreateBody(self.bodyDef)

        self.shape = b2PolygonShape(box=(width/2, length/2))
        self.body.CreateFixture(shape=self.shape, density=1)

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
        self.body.ApplyAngularImpulse(0.1 * self.body.inertia * \
                                      -self.body.angularVelocity, wake=True)

        currForwardNormal = self.getForwardVelocity()
        currForwardSpeed = currForwardNormal.Normalize()
        dragForce = -2 * currForwardSpeed
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
            desiredTorque = 15
        elif control[1] == -1:
            desiredTorque = -15
        else:
            return

        self.body.ApplyTorque(desiredTorque, wake=True)

    def draw(self, screen):
        for fixture in self.body.fixtures:
            shape = fixture.shape
            vertices = [self.body.transform * v * 10 for v in shape.vertices]
            vertices = [(v[0], 450-v[1]) for v in vertices]
            pygame.draw.polygon(screen, (255, 255, 255), vertices)

class Car:
    def __init__(self, world):
        self.bodyDef = b2BodyDef()
        self.bodyDef.type = b2_dynamicBody
        self.bodyDef.position = (50, 25)
        self.body = world.CreateBody(self.bodyDef)

        vertices = [(1.5, 0), (3, 2.5),
                    (2.8, 5.5), (1, 10),
                    (-1, 10), (-2.8, 5.5),
                    (-3, 2.5), (-1.5, 0)]
        self.shape = b2PolygonShape(vertices=vertices)
        self.fixture = self.body.CreateFixture(shape=self.shape, density=0.1)

        self.tires = []

        jointDef = b2RevoluteJointDef()
        jointDef.bodyA = self.body
        jointDef.enableLimit = True
        jointDef.lowerAngle = 0
        jointDef.upperAngle = 0
        jointDef.localAnchorB.SetZero()

        tireLength = 1.5
        tireWidth = 0.25

        tireFL = Tire(world, tireLength, tireWidth)
        jointDef.bodyB = tireFL.body
        jointDef.localAnchorA.Set(-3,8.5)
        self.flJoint = world.CreateJoint(jointDef)
        self.tires.append(tireFL)

        tireFR = Tire(world, tireLength, tireWidth)
        jointDef.bodyB = tireFR.body
        jointDef.localAnchorA.Set(3,8.5)
        self.frJoint = world.CreateJoint(jointDef)
        self.tires.append(tireFR)

        tireBL = Tire(world, tireLength, tireWidth)
        jointDef.bodyB = tireBL.body
        jointDef.localAnchorA.Set(-3,0.75)
        world.CreateJoint(jointDef)
        self.tires.append(tireBL)

        tireBR = Tire(world, tireLength, tireWidth)
        jointDef.bodyB = tireBR.body
        jointDef.localAnchorA.Set(3,0.75)
        world.CreateJoint(jointDef)
        self.tires.append(tireBR)

    def update(self, control):
        for tire in self.tires:
            tire.updateFriction()
        for tire in self.tires:
            tire.updateDrive(control)

        lockAngle = math.radians(35)
        turnSpeed = math.radians(320)
        turnPerTimeStep = turnSpeed / 60
        desiredAngle = 0

        if control[1] == 1:
            desiredAngle = -lockAngle
        elif control[1] == -1:
            desiredAngle = lockAngle

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
            vertices = [self.body.transform * v * 10 for v in shape.vertices]
            vertices = [(v[0], 450-v[1]) for v in vertices]
            pygame.draw.polygon(screen, (0, 25, 156), vertices)

        for tire in self.tires:
            tire.draw(screen)

def draw(body):
        for fixture in body.fixtures:
            shape = fixture.shape
            vertices = [body.transform * v * 10 for v in shape.vertices]
            vertices = [(v[0], 450-v[1]) for v in vertices]
            pygame.draw.polygon(screen, (125, 25, 125), vertices)

if __name__ == "__main__":
    pygame.init()
    screen = pygame.display.set_mode((1200, 800))
    clock = pygame.time.Clock()

    running = True

    world = b2World(gravity=(0,0))

    groundBody = world.CreateBody()
    groundShape = b2PolygonShape(box=(1200, 800))
    groundFixtureDef = b2FixtureDef(shape=groundShape)
    groundFixtureDef.isSensor = True
    groundFixture = groundBody.CreateFixture(groundFixtureDef)

o   wallBody = world.CreateBody(position=(0,-30))
    wallShape = b2PolygonShape(box=(125, 2.5))
    wallFixtureDef = b2FixtureDef(shape=wallShape)
    wallFixture = wallBody.CreateFixture(wallFixtureDef)

    car = Car(world)

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
        draw(wallBody)
        world.Step(timestep, vel_iters, pos_iters)
        car.update((forwardCommand, turnCommand))
        pygame.display.flip()
        clock.tick(60)
    pygame.quit()
