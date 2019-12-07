from enum import Enum
import numpy as np
import random

colorBody = (192, 192, 192)
colorDirectionLine = (25, 118, 210)
size = 15
velocity = 1
turnSpeed = np.pi / 36
communicationRange = 60

preferedDistance = 40 #Bugged for <= 2 * size
maxAngleError = np.pi / 36


class State(Enum):
    WAIT_TO_MOVE = 1
    MOVING = 2
    JOINED_SHAPE = 3


class Kilobot:
    def __init__ (self, renderer, startPosition, startDirection, bitMapArray, bitMapScalingFactor, gradientVal):
        self.renderer = renderer
        self.x, self.y = startPosition
        self.direction = startDirection
        #self.bitMapArray = bitMapArray.repeat(bitMapScalingFactor,0).repeat(bitMapScalingFactor, 1)
        self.bitMapArray = bitMapArray
        self.bitMapScalingFactor = bitMapScalingFactor
        self.gradientVal = gradientVal
        self.state = State.WAIT_TO_MOVE
        self.id = random.random()  # very idealistic

    def _getNeighbors(self, kilobots):
        neighbors = []
        for bot in kilobots:
            if bot is self:
                continue
            xDiff = self.x - bot.x
            yDiff = self.y - bot.y
            dist = np.sqrt(xDiff**2 + yDiff**2)

            if dist <= communicationRange:
                neighbors.append(bot)
        return neighbors

    def timestep(self, deltaTime, kilobots):
        # Calculate gradient value
        neighbors = self._getNeighbors(kilobots)

        neighborGradients = [None] * len(neighbors)
        for i in range(len(neighbors)):
            neighborGradients[i] = neighbors[i].gradientVal
        if len(neighborGradients) == 0:
            minNeighborGradient = np.inf
            maxNeighborGradient = np.inf
        else:
            minNeighborGradient = min(neighborGradients)
            maxNeighborGradient = max(neighborGradients)
        self.gradientVal = minNeighborGradient + 1

        if self.state == State.WAIT_TO_MOVE:
            neighborIdsWithSameGradient = []
            for i in range(len(neighbors)):
                if neighbors[i].gradientVal == self.gradientVal:
                    neighborIdsWithSameGradient.append(neighbors[i].id)
            if len(neighborIdsWithSameGradient) == 0:
                maxNeighborIdWithSameGradient = np.inf
            else:
                maxNeighborIdWithSameGradient = max(neighborIdsWithSameGradient)

            if self.gradientVal > maxNeighborGradient or (self.gradientVal == maxNeighborGradient and self.id > maxNeighborIdWithSameGradient):
                self.state = State.MOVING

        elif self.state == State.MOVING:
            nearestRobotX, nearestRobotY = self._findClosest(deltaTime, kilobots)
            bitMapVal = 0
            xDim = self.bitMapArray.shape[0]*self.bitMapScalingFactor
            yDim = self.bitMapArray.shape[1]*self.bitMapScalingFactor
            if self.x >= 0 and self.y >= 0 and self.x < xDim and self.y  < yDim:
                x = int(self.x/self.bitMapScalingFactor)
                y = int(self.y/self.bitMapScalingFactor)
                bitMapVal = self.bitMapArray[x,y]
                print(x,y, bitMapVal)
                isOnEdge = self.isOnEdge(bitMapVal, deltaTime)
            if bitMapVal == 0:  #move into position
                self._move(deltaTime, nearestRobotX, nearestRobotY)
            elif bitMapVal ==1 and isOnEdge == False:
                self._move(deltaTime, nearestRobotX, nearestRobotY)

        elif self.state == State.JOINED_SHAPE:
            pass  # do nothing

    def isOnEdge(self, bitMapVal, dt):
        xfuture = self.x + velocity*dt*np.cos(self.direction)
        yfuture = self.y + velocity*dt*np.sin(self.direction)

        if bitMapVal == 1:
            xDim = self.bitMapArray.shape[0]*self.bitMapScalingFactor
            yDim = self.bitMapArray.shape[1]*self.bitMapScalingFactor
            if xfuture >= 0 and yfuture >= 0 and xfuture < xDim and yfuture < yDim:
                yfuture = int(yfuture/self.bitMapScalingFactor)
                xfuture = int(xfuture/self.bitMapScalingFactor)
                nextVal = self.bitMapArray[xfuture,yfuture]
                print(xfuture, yfuture, nextVal)
                if nextVal == 0:
                    return True #we are on the edge stop fucking MOVING
            elif xfuture < 0 or yfuture < 0 or xfuture >= xDim or yfuture >= yDim:
                return True
        return False

    def _findClosest(self, deltaTime, kilobots):
        rmax = 100
        nearestX = 1
        nearestY = 1

        for bot in kilobots:
            if bot is not self:
                r = np.sqrt( (self.x - bot.x)**2 + (self.y - bot.y)**2 )
                if r < rmax:
                    rmax = r
                    nearestX = bot.x
                    nearestY = bot.y
        return nearestX, nearestY

    def _move(self, deltaTime, nearestX, nearestY):
        dx = self.x - nearestX
        dy = self.y - nearestY
        rVector = np.array([dx, dy, 0])
        w = np.cross(rVector, np.array([0, 0, 1]))
        tempVector = w / np.sqrt( np.dot(w, w) ) +  \
                (preferedDistance - np.sqrt(dx**2 + dy**2) ) / \
                ( preferedDistance - 2 * size ) * rVector / np.sqrt(np.dot(rVector,rVector))
        preferedDirectionVector = tempVector / np.sqrt( np.dot(tempVector, tempVector) )

        directionVector = np.array([np.cos(self.direction), np.sin(self.direction), 0])
        choiceVector = np.dot(preferedDirectionVector, directionVector)

        if choiceVector < np.cos(maxAngleError):
            w = np.cross(directionVector, np.array([0, 0, 1]))
            tempVector = np.dot(w,preferedDirectionVector)
            self.direction -= turnSpeed * tempVector / np.sqrt( np.dot(tempVector, tempVector) )
        else:
            self.x += velocity * deltaTime * np.cos(self.direction)
            self.y += velocity * deltaTime * np.sin(self.direction)


    def draw(self):
        position = (int(self.x), int(self.y))
        directionLineTarget = (
            int(self.x + np.cos(self.direction) * size),
            int(self.y + np.sin(self.direction) * size),
        )
        self.renderer.drawCircle(colorBody, position, size)
        self.renderer.drawLine(colorDirectionLine, position, directionLineTarget, size/5)

        if self.state == State.WAIT_TO_MOVE:
            textColor = (255, 0, 0)
        elif self.state == State.MOVING:
            textColor = (255, 255, 0)
        elif self.state == State.JOINED_SHAPE:
            textColor = (0, 255, 0)
        self.renderer.drawText(textColor, str(self.gradientVal), position)




class KilobotOrigin(Kilobot):
    def __init__(self, renderer, startPosition, startDirection, bitMapArray, bitMapScalingFactor, gradientVal):
        Kilobot.__init__(self, renderer, startPosition, startDirection, bitMapArray, bitMapScalingFactor, gradientVal)
        self.state = State.JOINED_SHAPE

    def timestep(self, deltaTime, kilobots):
        pass
