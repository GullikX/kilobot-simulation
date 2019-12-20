from __future__ import division
from enum import IntEnum
import heapq
from math import sqrt
import numpy as np
import time

from helpers import calcOverlappingA, isInsideShape
from spatialmap import SpatialMap

colorBody = (
    (229, 57, 53),   # Waiting
    (66, 175, 80),   # Moving
    (25, 118, 210),  # Joined shape
)
colorDirectionLine = (192, 192, 192)

size = 15
velocity = 1
turnSpeed = np.pi / 20
communicationRange = 170
neighborUpdateInterval = 5

preferedDistance = 32 #Bugged for <= 2 * size
maxAngleError = np.pi / 20
gradientCommunicationRange = preferedDistance + 10
noiseStdDev = 5
directionNoiseStdDev = 0.1
stoppingTimesteps = 25
nScorePoints = 100

spatialMapGridSize = 7  # [mm]
spatialMapCells = 3000
areaSize = spatialMapGridSize * spatialMapCells - 2*communicationRange  # [mm]
areaBoundaryColor = (189, 189, 189)


class State(IntEnum):
    WAIT_TO_MOVE = 0
    MOVING = 1
    JOINED_SHAPE = 2


class Kilobot:
    bitMapArray = []
    scalingFactor = 0
    spatialMap = SpatialMap(spatialMapGridSize, spatialMapCells)
    def __init__ (self, renderer, startPosition, startDirection, bitMapArray, scalingFactor, gradientVal, nRobots):
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
        Kilobot.spatialMap.addEntry(self, self.pActual)
        self.initialWaitTime = nRobots

    def _getMovePriority(self):
        if self.state != State.MOVING:
            return -np.inf
        angle = np.arctan2(self.pos[1], self.pos[0])
        if angle < 0:
            angle += 2*np.pi
        return -angle

    def _getNeighbors(self, kilobots):
        self.gradNeighbors = Kilobot.spatialMap.getNeighbors(self, self.pActual, gradientCommunicationRange)
        self.comNeighbors = Kilobot.spatialMap.getNeighbors(self, self.pActual, communicationRange)


    def timestep(self, iTimestep, deltaTime, kilobots):
        if self.state == State.JOINED_SHAPE:
            return  # do nothing

        self._localize()

        # Calculate gradient value
        if iTimestep % neighborUpdateInterval == 0:  # Increase performance by not updating each frame
            self._getNeighbors(kilobots)
        [neighborGradients, maxNeighborGradient] = self._getNeighborGradients()

        if self.state == State.WAIT_TO_MOVE and iTimestep > self.initialWaitTime:
            nNeighborsMoving = 0
            for neighbor in self.comNeighbors:
                if neighbor.state == State.MOVING:
                    nNeighborsMoving += 1
            if self.gradientVal < np.inf and self.gradientVal >= maxNeighborGradient and nNeighborsMoving == 0:
                self.state = State.MOVING
        elif self.state == State.MOVING:
            Kilobot.spatialMap.removeEntry(self, self.pActual)
            if np.isnan(self.pos).any():
                self._edgeFollow(deltaTime, None)
            else:
                if len(self.comNeighbors) > 0:
                    neighborMovePriorities = [None] * len(self.comNeighbors)
                    for i in range(len(self.comNeighbors)):
                        neighborMovePriorities[i] = self.comNeighbors[i]._getMovePriority()
                    if self._getMovePriority() > max(neighborMovePriorities):
                        closestRobot = self._findClosestRobot(deltaTime)
                        self._edgeFollow(deltaTime,closestRobot)

                    closestRobot = self._findClosestRobot(deltaTime)
                    dirV = np.array([np.cos(self.direction), np.sin(self.direction)])
                    v = closestRobot.pos - self.pos
                    d = np.dot(dirV, v)/np.linalg.norm(v)

                    justSwitchedOrbit = (d > np.sin(np.pi/6))
                    isInsideShape = self._isInsideShape()
                    hasEnteredShapeBefore = (self.enteredShapeTimestep > 0)
                    hasBeenInsideShapeLongEnough = ((iTimestep - self.enteredShapeTimestep) > stoppingTimesteps)
                    orbitingNeighborWithHigherOrEqualGradient = (closestRobot.gradientVal >= self.gradientVal)

                    if isInsideShape and not hasEnteredShapeBefore:
                            self.enteredShapeTimestep = iTimestep
                    elif (
                            (hasEnteredShapeBefore and not isInsideShape and hasBeenInsideShapeLongEnough) or
                            (isInsideShape and orbitingNeighborWithHigherOrEqualGradient and justSwitchedOrbit)
                        ):
                        self.state = State.JOINED_SHAPE
                else:
                    self._edgeFollow(deltaTime, None)

            Kilobot.spatialMap.addEntry(self, self.pActual)

    def _isInsideShape(self):
        return isInsideShape(Kilobot.bitMapArray, Kilobot.scalingFactor, self.pos)

    def calculateScore(self):
        # Generate points on the robot's perimeter, calculate the fraction of them inside the shape
        theta = np.linspace(0, 2*np.pi, nScorePoints)
        xValues = self.pos[0] + size * np.cos(theta)
        yValues = self.pos[1] + size * np.sin(theta)

        nPointsInsideShape = 0
        for i in range(nScorePoints):
            pos = np.array((xValues[i], yValues[i]))
            nPointsInsideShape += int(isInsideShape(Kilobot.bitMapArray, Kilobot.scalingFactor, pos))

        return nPointsInsideShape / nScorePoints

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
        self._wrap()

    def _wrap(self):
        if self.pActual[0] < areaSize/2:
            self.pActual[0] += areaSize
        if self.pActual[1] < areaSize/2:
            self.pActual[1] += areaSize
        if self.pActual[0] > areaSize/2:
            self.pActual[0] -= areaSize
        if self.pActual[1] > areaSize/2:
            self.pActual[1] -= areaSize

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

        self.renderer.drawCircle(colorBody[self.state], position, size)
        self.renderer.drawLine(colorDirectionLine, position, directionLineTarget, size/4)
        if self.state == State.MOVING:
            self.renderer.drawCircle(colorBody[self.state], position, gradientCommunicationRange, width=1)

        self.renderer.drawRectangle(
                areaBoundaryColor,
                (-areaSize/2,)*2,
                (areaSize,)*2,
                width=3,
        )
        #locError = self.pos - self.pActual
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

    def calcOverlappingA(self):
        return calcOverlappingA(self.pActual, size, nScorePoints, Kilobot.bitMapArray, Kilobot.scalingFactor)

    def resetSpatialMap(self):
        Kilobot.spatialMap = SpatialMap(spatialMapGridSize, spatialMapCells)

    def getNoiseVal(self):
        return  "noise=" + str(noiseStdDev) + "-" + "dirNoise=" + str(directionNoiseStdDev)

class KilobotOrigin(Kilobot):
    def __init__(self, renderer, startPosition, startDirection, bitMapArray, scalingFactor, gradientVal, nRobots):
        Kilobot.__init__(self, renderer, startPosition, startDirection, bitMapArray, scalingFactor, gradientVal, nRobots)
        self.state = State.JOINED_SHAPE
        self.pos = self.pActual
        self.sensorError = np.array([0, 0])

    def timestep(self, iTimestep, deltaTime, kilobots):
        pass
