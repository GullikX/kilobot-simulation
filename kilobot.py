from __future__ import division
from enum import Enum
import heapq
from math import sqrt
import numpy as np
import time
colorBodyWaiting = (229, 57, 53)
colorBodyMoving = (66, 175, 80)
colorBodyJoinedShape = (25, 118, 210)
colorDirectionLine = (192, 192, 192)

size = 15
velocity = 1
turnSpeed = np.pi / 30
communicationRange = 100
neighborUpdateInterval = 5

preferedDistance = 32 #Bugged for <= 2 * size
maxAngleError = np.pi / 30
gradientCommunicationRange = preferedDistance + 10
noiseStdDev = 1


class State(Enum):
    WAIT_TO_MOVE = 1
    MOVING = 2
    JOINED_SHAPE = 3


class Kilobot:
    bitMapArray = []
    scalingFactor = 0
    def __init__ (self, renderer, startPosition, startDirection, bitMapArray, scalingFactor, gradientVal):
        self.renderer = renderer
        Kilobot.bitMapArray = np.transpose(np.flip(bitMapArray, 0))
        Kilobot.scalingFactor = scalingFactor

        self.pActual = np.asarray(startPosition).reshape(-1)
        self.pos = np.array([np.nan, np.nan]).T
        self.direction = startDirection
        self.gradientVal = gradientVal
        self.state = State.WAIT_TO_MOVE
        self.neighbors = []
        self.sensorError = np.random.normal(0, noiseStdDev)

    def _getMovePriority(self):
        if self.state != State.MOVING:
            return -np.inf
        angle = np.arctan2(self.pos[1], self.pos[0])
        if angle < 0:
            angle += 2*np.pi
        return -angle

    def _getNeighbors(self, kilobots):
        neighbors = []
        comNeighbours = []
        for bot in kilobots:
            if bot is self:
                continue
            diff = np.sum((self.pActual - bot.pActual)**2)
            if diff <= gradientCommunicationRange**2:
                neighbors.append(bot)
        return neighbors

    def timestep(self, iTimestep, deltaTime, kilobots):
        self._localize()

        # Calculate gradient value
        if iTimestep % neighborUpdateInterval == 0:  # Increase performance by not updating each frame
            self.neighbors = self._getNeighbors(kilobots)
        [neighborGradients, maxNeighborGradient] = self._getNeighborGradients()

        if self.state == State.WAIT_TO_MOVE:
            if (self.gradientVal >= maxNeighborGradient) and len(self.neighbors) < 4:
                self.state = State.MOVING
        elif self.state == State.MOVING:
            if np.isnan(self.pos).any():
                self._move(deltaTime, None)
                return
            neighborMovePriorities = [None] * len(self.neighbors)
            for i in range(len(self.neighbors)):
                neighborMovePriorities[i] = self.neighbors[i]._getMovePriority()
            if len(self.neighbors) == 0 or self._getMovePriority() > max(neighborMovePriorities):
                closestRobot = self._findClosestRobot(deltaTime, self.neighbors)
                if self._isInsideShape():
                    if self._isOnEdge(deltaTime) or closestRobot.gradientVal >= self.gradientVal:
                        self.state = State.JOINED_SHAPE
                        self.startTime = np.inf
                    else:
                        self._move(deltaTime,closestRobot)
                else:
                    self._move(deltaTime,closestRobot)

        elif self.state == State.JOINED_SHAPE:
            pass  # do nothing

    def _isInsideShape(self):
        dim = np.multiply(Kilobot.bitMapArray.shape,Kilobot.scalingFactor)
        if np.all(self.pos >= 0) and np.all(self.pos < dim):
            p = self.pos/Kilobot.scalingFactor
            bitMapVal = Kilobot.bitMapArray[p[0].astype(int), p[1].astype(int)]
            return bool(bitMapVal)
        return False

    def _isOnEdge(self, dt):
        xfuture = self.pos[0] + velocity*dt*np.cos(self.direction)
        yfuture = self.pos[1] + velocity*dt*np.sin(self.direction)
        xDim = Kilobot.bitMapArray.shape[0]*Kilobot.scalingFactor
        yDim = Kilobot.bitMapArray.shape[1]*Kilobot.scalingFactor

        if xfuture >= 0 and yfuture >= 0 and xfuture < xDim and yfuture < yDim:
            yfuture2 = int(yfuture/Kilobot.scalingFactor)
            xfuture2 = int(xfuture/Kilobot.scalingFactor)
            nextVal = Kilobot.bitMapArray[xfuture2,yfuture2]
            if nextVal == 0:
               return True #we are on the edge stop fucking MOVING
        elif xfuture < 0 or yfuture < 0 or xfuture >= xDim or yfuture >= yDim:
            return True

    def _findClosestRobot(self, deltaTime, kilobots):
        rmax = np.inf
        closestBot = None
        for bot in kilobots:
            if bot is not self:
                rSquared = np.sum((self.pos - bot.pos)**2)
                if rSquared < rmax:
                    rmax = rSquared
                    closestBot = bot

        return closestBot

    def _move(self, deltaTime, nearestRobot):
        if nearestRobot is None:
            dirV = np.array([np.cos(self.direction), np.sin(self.direction)])
            self.pActual += velocity * deltaTime * dirV
            bot = self._findClosestRobot(deltaTime, self.neighbors)
            self.collision(bot)
            return

        diff = self.pos - nearestRobot.pos
        rVector = np.append(diff, 0)
        w = np.cross(rVector, np.array([0, 0, 1]))
        tempVector = w / np.sqrt( np.dot(w, w) ) +  \
                (preferedDistance - np.sqrt(np.sum(diff**2))) / \
                ( preferedDistance - 2 * size ) * rVector / np.sqrt(np.dot(rVector,rVector))
        preferedDirectionVector = tempVector / np.sqrt( np.dot(tempVector, tempVector) )

        directionVector = np.array([np.cos(self.direction),np.sin(self.direction), 0])
        choiceVector = np.dot(preferedDirectionVector, directionVector)

        if choiceVector < np.cos(maxAngleError):
            w = np.cross(directionVector, np.array([0, 0, 1]))
            tempVector = np.dot(w,preferedDirectionVector)
            self.direction -= turnSpeed * deltaTime * tempVector / np.sqrt( np.dot(tempVector, tempVector) )
        else:
            dirV = np.array([np.cos(self.direction), np.sin(self.direction)])
            self.pActual += velocity * deltaTime * dirV
            bot = self._findClosestRobot(deltaTime, self.neighbors)
            self.collision(bot)

    def _localize(self):
        if len(self.neighbors) == 0:
            self.pos = np.array([np.nan, np.nan]).T
        minGradientNeighbor = None
        minGradientVal = np.inf
        for neighbor in self.neighbors:
            if (neighbor.state == State.WAIT_TO_MOVE or neighbor.state == State.JOINED_SHAPE) and neighbor.gradientVal < minGradientVal:
                minGradientNeighbor = neighbor
                minGradientVal = neighbor.gradientVal
        if minGradientNeighbor is None:
            self.pos = np.array([np.nan, np.nan]).T
        else:
            self.pos = minGradientNeighbor.pos + (self.pActual - minGradientNeighbor.pActual) + self.sensorError

    def draw(self):
        position = (int(self.pActual[0]), int(self.pActual[1]))
        directionLineTarget = (
            int(self.pActual[0] + np.cos(self.direction) * size),
            int(self.pActual[1] + np.sin(self.direction) * size),
        )


        if self.state == State.WAIT_TO_MOVE:
            colorBody = colorBodyWaiting
        elif self.state == State.MOVING:
            colorBody = colorBodyMoving
        elif self.state == State.JOINED_SHAPE:
            colorBody = colorBodyJoinedShape

        self.renderer.drawCircle(colorBody, position, size)
        self.renderer.drawLine(colorDirectionLine, position, directionLineTarget, size/4)
        locError = self.pos - self.pActual
        self.renderer.drawText((255, 255, 255), f"{locError[0]:.1f}", position)

    def _getNeighborGradients(self):
        neighborGradients = [None] * len(self.neighbors)
        for i in range(len(self.neighbors)):
            neighborGradients[i] = self.neighbors[i].gradientVal
        if len(neighborGradients) == 0:
            minNeighborGradient = np.inf
            maxNeighborGradient = np.inf
        else:
            minNeighborGradient = min(neighborGradients)
            maxNeighborGradient = max(neighborGradients)
        self.gradientVal = minNeighborGradient + 1

        return [neighborGradients, maxNeighborGradient]

    def collision(self, bot):
        if bot is not None:
            normDist = np.sqrt(np.sum((self.pActual - bot.pActual)**2))
            normV = (bot.pActual - self.pActual)/normDist
            if normDist < 2*size:
                self.pActual += normV*(size - normDist)*0.1


class KilobotOrigin(Kilobot):
    def __init__(self, renderer, startPosition, startDirection, bitMapArray, scalingFactor, gradientVal):
        Kilobot.__init__(self, renderer, startPosition, startDirection, bitMapArray, scalingFactor, gradientVal)
        self.state = State.JOINED_SHAPE
        self.pos = self.pActual
        self.sensorError = 0


    def timestep(self, iTimestep, deltaTime, kilobots):
        pass
