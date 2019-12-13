from enum import Enum
import numpy as np

colorBodyWaiting = (229, 57, 53)
colorBodyMoving = (66, 175, 80)
colorBodyJoinedShape = (25, 118, 210)
colorDirectionLine = (192, 192, 192)

size = 15
velocity = 1
turnSpeed = np.pi / 30
communicationRange = 100
neighborUpdateInterval = 5

preferedDistance = 31 #Bugged for <= 2 * size
maxAngleError = np.pi / 30
gradientCommunicationRange = preferedDistance + 5


class State(Enum):
    WAIT_TO_MOVE = 1
    MOVING = 2
    JOINED_SHAPE = 3


class Kilobot:
    bitMapArray = []
    bitMapScalingFactor = 0
    botsPartOfShape = []
    def __init__ (self, renderer, startPosition, startDirection, bitMapArray, bitMapScalingFactor, gradientVal):
        self.renderer = renderer

        self.x = startPosition[0,0]
        self.y = startPosition[0,1]
        self.direction = startDirection
        Kilobot.bitMapArray = np.transpose(np.flip(bitMapArray, 0))
        Kilobot.bitMapScalingFactor = bitMapScalingFactor
        self.gradientVal = gradientVal
        self.state = State.WAIT_TO_MOVE
        self.neighbors = []

    def _getMovePriority(self):
        if self.state != State.MOVING:
            return -np.inf
        angle = np.arctan2(self.y, self.x)
        if angle < 0:
            angle += 2*np.pi
        return -angle

    def _getNeighbors(self, kilobots):
        neighbors = []
        for bot in kilobots:
            if bot is self:
                continue
            xDiff = self.x - bot.x
            yDiff = self.y - bot.y
            distSquared = xDiff**2 + yDiff**2

            if distSquared <= gradientCommunicationRange**2:
                neighbors.append(bot)
        return neighbors

    def timestep(self, iTimestep, deltaTime, kilobots):
        # Calculate gradient value
        if iTimestep % neighborUpdateInterval == 0:  # Increase performance by not updating each frame
            self.neighbors = self._getNeighbors(kilobots)
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

        if self.state == State.WAIT_TO_MOVE:
            if (self.gradientVal >= maxNeighborGradient) and len(self.neighbors) < 4:
                self.state = State.MOVING

        elif self.state == State.MOVING:
            neighborMovePriorities = [None] * len(self.neighbors)
            for i in range(len(self.neighbors)):
                neighborMovePriorities[i] = self.neighbors[i]._getMovePriority()
            if len(self.neighbors) == 0 or self._getMovePriority() > max(neighborMovePriorities):
                closestRobot = self._findClosestRobot(deltaTime, self.neighbors)
                if self._isInsideShape():
                    if self._isOnEdge(deltaTime) or closestRobot.gradientVal >= self.gradientVal:
                        self.state = State.JOINED_SHAPE
                        Kilobot.botsPartOfShape.append(self)
                        self.startTime = np.inf
                    else:
                        self._move(deltaTime, closestRobot)
                else:
                    self._move(deltaTime, closestRobot)

        elif self.state == State.JOINED_SHAPE:
            pass  # do nothing

    def _isInsideShape(self):
        xDim = Kilobot.bitMapArray.shape[0]*Kilobot.bitMapScalingFactor
        yDim = Kilobot.bitMapArray.shape[1]*Kilobot.bitMapScalingFactor
        if self.x >= 0 and self.y >= 0 and self.x < xDim and self.y  < yDim:
            x = int(self.x/Kilobot.bitMapScalingFactor)
            y = int(self.y/Kilobot.bitMapScalingFactor)
            bitMapVal = Kilobot.bitMapArray[x,y]
            return bool(bitMapVal)
        return False

    def _isOnEdge(self, dt):
        xfuture = self.x + velocity*dt*np.cos(self.direction)
        yfuture = self.y + velocity*dt*np.sin(self.direction)

        xDim = Kilobot.bitMapArray.shape[0]*Kilobot.bitMapScalingFactor
        yDim = Kilobot.bitMapArray.shape[1]*Kilobot.bitMapScalingFactor
        if xfuture >= 0 and yfuture >= 0 and xfuture < xDim and yfuture < yDim:
            yfuture2 = int(yfuture/Kilobot.bitMapScalingFactor)
            xfuture2 = int(xfuture/Kilobot.bitMapScalingFactor)
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
                rSquared = (self.x - bot.x)**2 + (self.y - bot.y)**2
                if rSquared < rmax:
                    rmax = rSquared
                    closestBot = bot
        return closestBot

    def _move(self, deltaTime, nearestRobot):
        if nearestRobot is None:
            self.x += velocity * deltaTime * np.cos(self.direction)
            self.y += velocity * deltaTime * np.sin(self.direction)
            return

        dx = self.x - nearestRobot.x
        dy = self.y - nearestRobot.y
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
            self.direction -= turnSpeed * deltaTime * tempVector / np.sqrt( np.dot(tempVector, tempVector) )
        else:
            self.x += velocity * deltaTime * np.cos(self.direction)
            self.y += velocity * deltaTime * np.sin(self.direction)


    def draw(self):
        position = (int(self.x), int(self.y))
        directionLineTarget = (
            int(self.x + np.cos(self.direction) * size),
            int(self.y + np.sin(self.direction) * size),
        )

        if self.state == State.WAIT_TO_MOVE:
            colorBody = colorBodyWaiting
        elif self.state == State.MOVING:
            colorBody = colorBodyMoving
        elif self.state == State.JOINED_SHAPE:
            colorBody = colorBodyJoinedShape

        self.renderer.drawCircle(colorBody, position, size)
        self.renderer.drawLine(colorDirectionLine, position, directionLineTarget, size/4)
        #self.renderer.drawText(textColor, str(self.gradientVal), position)



class KilobotOrigin(Kilobot):
    def __init__(self, renderer, startPosition, startDirection, bitMapArray, bitMapScalingFactor, gradientVal):
        Kilobot.__init__(self, renderer, startPosition, startDirection, bitMapArray, bitMapScalingFactor, gradientVal)
        self.state = State.JOINED_SHAPE
        Kilobot.botsPartOfShape.append(self)
    def timestep(self, iTimestep, deltaTime, kilobots):
        pass
