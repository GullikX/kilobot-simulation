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
noiseStdDev = 5
directionNoiseStdDev = 0.1
stoppingTimesteps = 25


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
        self.gradNeighbors = []
        self.comNeighbors = []
        self.sensorError = np.array([np.random.normal(0, noiseStdDev), np.random.normal(0, noiseStdDev)])
        self.enteredShapeTimestep = -1

    def _getMovePriority(self):
        if self.state != State.MOVING:
            return -np.inf
        angle = np.arctan2(self.pos[1], self.pos[0])
        if angle < 0:
            angle += 2*np.pi
        return -angle

    def _getNeighbors(self, kilobots):
        gradNeighbors = []
        comNeighbors = []
        for bot in kilobots:
            if bot is self:
                continue
            diff = np.sum((self.pActual - bot.pActual)**2)
            if diff <= communicationRange**2:
                comNeighbors.append(bot)
                if diff <= gradientCommunicationRange**2:
                    gradNeighbors.append(bot)
        self.gradNeighbors = gradNeighbors
        self.comNeighbors = comNeighbors

    def timestep(self, iTimestep, deltaTime, kilobots):
        self._localize()

        # Calculate gradient value
        if iTimestep % neighborUpdateInterval == 0:  # Increase performance by not updating each frame
            self._getNeighbors(kilobots)
        [neighborGradients, maxNeighborGradient] = self._getNeighborGradients()

        if self.state == State.WAIT_TO_MOVE:
            nNeighborsMoving = 0
            for neighbor in self.comNeighbors:
                if neighbor.state == State.MOVING:
                    nNeighborsMoving += 1
            if self.gradientVal < np.inf and self.gradientVal >= maxNeighborGradient and nNeighborsMoving == 0:
                self.state = State.MOVING
        elif self.state == State.MOVING:
            if np.isnan(self.pos).any():
                self._edgeFollow(deltaTime, None)
                return
            neighborMovePriorities = [None] * len(self.comNeighbors)
            for i in range(len(self.comNeighbors)):
                neighborMovePriorities[i] = self.comNeighbors[i]._getMovePriority()
            if len(self.comNeighbors) == 0 or self._getMovePriority() > max(neighborMovePriorities):
                closestRobot = self._findClosestRobot(deltaTime)
                self._edgeFollow(deltaTime,closestRobot)

            isInsideShape = self._isInsideShape()
            if isInsideShape and self.enteredShapeTimestep == -1:
                    self.enteredShapeTimestep = iTimestep
            elif ((not isInsideShape and not self.enteredShapeTimestep == -1 and
                     (iTimestep - self.enteredShapeTimestep) > stoppingTimesteps) or
                    (isInsideShape and closestRobot.gradientVal >= self.gradientVal)):
                self.state = State.JOINED_SHAPE

        elif self.state == State.JOINED_SHAPE:
            pass  # do nothing

    def _isInsideShape(self):
        dim = np.multiply(Kilobot.bitMapArray.shape,Kilobot.scalingFactor)
        if np.all(self.pos >= 0) and np.all(self.pos < dim):
            p = self.pos/Kilobot.scalingFactor
            bitMapVal = Kilobot.bitMapArray[p[0].astype(int), p[1].astype(int)]
            return bool(bitMapVal)
        return False

    def _findClosestRobot(self, deltaTime):
        rmax = np.inf
        closestBot = None
        for bot in self.comNeighbors:
            if bot is not self:
                rSquared = np.sum((self.pActual - bot.pActual)**2)
                if rSquared < rmax:
                    rmax = rSquared
                    closestBot = bot

        return closestBot

    def _edgeFollow(self, deltaTime, nearestRobot):
        if nearestRobot is None:
            self.move(deltaTime)
            return

        diff = self.pActual - nearestRobot.pActual
        rVector = np.append(diff, 0)
        w = np.cross(rVector, np.array([0, 0, 1]))
        wNorm = w/np.linalg.norm(w)
        rVectorNorm = rVector/np.linalg.norm(rVector)
        s =(preferedDistance-np.linalg.norm(diff))/(preferedDistance-2*size)
        tempVector = wNorm + s*rVectorNorm
        preferedDirectionVector = tempVector / np.linalg.norm(tempVector)

        directionVector = np.array([np.cos(self.direction),np.sin(self.direction), 0])
        choiceVector = np.dot(preferedDirectionVector, directionVector)

        if choiceVector < np.cos(maxAngleError):
            w = np.cross(directionVector, np.array([0, 0, 1]))
            tempVector = np.dot(w,preferedDirectionVector)
            self.direction -= turnSpeed * deltaTime * tempVector/np.linalg.norm(tempVector)
        else:
            self.move(deltaTime)

    def move(self, deltaTime):
        dirV = np.array([
            np.cos(self.direction) + np.random.normal(0, directionNoiseStdDev),
            np.sin(self.direction) + np.random.normal(0, directionNoiseStdDev),
        ])
        self.pActual += velocity * deltaTime * dirV
        bot = self._findClosestRobot(deltaTime)
        self.collision(bot)

    def _localize(self):
        posApproximations = []
        for neighbor in self.comNeighbors:
            if (neighbor.state == State.WAIT_TO_MOVE or neighbor.state == State.JOINED_SHAPE) and neighbor.gradientVal < self.gradientVal:
                posApproximations.append(neighbor.pos + (self.pActual - neighbor.pActual) + self.sensorError)
        if len(posApproximations) == 0:
            self.pos = np.array([np.nan, np.nan]).T
            return
        posApproximations = np.array(posApproximations)
        self.pos = np.mean(posApproximations, axis=0)

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
        if self.state == State.MOVING:
            self.renderer.drawCircle(colorBody, position, gradientCommunicationRange, width=1)
        locError = self.pos - self.pActual
        #self.renderer.drawText((255, 255, 255), f"{locError[0]:.1f}", position)

    def _getNeighborGradients(self):
        neighborGradients = [None] * len(self.gradNeighbors)
        for i in range(len(self.gradNeighbors)):
            neighborGradients[i] = self.gradNeighbors[i].gradientVal
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
    def checkState(self):
        inside = self_isInsideShape()
        if inside == True:
            m = np.array([x + size,y], [x - size, y], [x, y+size], x, y-size)
            m = m/Kilobot.scalingFactor
            for i in range(4):
                bitMapVal = bitMapArray(m[0,i].astype(int), m[1,i].astype(int))
                if bool(bitMapVal):
                    return False    #outside
        return True

class KilobotOrigin(Kilobot):
    def __init__(self, renderer, startPosition, startDirection, bitMapArray, scalingFactor, gradientVal):
        Kilobot.__init__(self, renderer, startPosition, startDirection, bitMapArray, scalingFactor, gradientVal)
        self.state = State.JOINED_SHAPE
        self.pos = self.pActual
        self.sensorError = np.array([0, 0])


    def timestep(self, iTimestep, deltaTime, kilobots):
        pass
