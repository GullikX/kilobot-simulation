import numpy as np
import csv

colorBody = (192, 192, 192)
colorDirectionLine = (25, 118, 210)
size =15
velocity = 1
turnSpeed = np.pi / 36
communicationRange = 60

preferedDistance = 50 #Bugged for <= 2 * size
maxAngleError = np.pi / 36

class Kilobot:
    counter = 0;
    def __init__ (self, renderer, startPosition, startDirection, bitMapArray, bitMapScalingFactor):
        self.renderer = renderer
        self.x, self.y = startPosition
        self.direction = startDirection
        self.bitMapArray = bitMapArray
        self.bitMapScalingFactor = bitMapScalingFactor
        self.gradientVal = np.inf    #See paper
        if Kilobot.counter < 4:
            if Kilobot.counter == 0:
                self.gradientVal = 0
            else:
                self.gradientVal = 1
            self.moveVal = 3
        else:
            self.moveVal = 0
        Kilobot.counter += 1

    def _setGradient(self, kilobots):
        minimumGradient = np.inf
        for bot in kilobots:
            if bot is self:
                continue
            xDiff = self.x - bot.x
            yDiff = self.y - bot.y
            dist = np.sqrt(xDiff**2 + yDiff**2)

            if dist <= communicationRange and bot.gradientVal < minimumGradient:
                minimumGradient = bot.gradientVal
        self.gradientVal = 1 + minimumGradient

    def timestep(self, deltaTime, kilobots):
        self._setGradient(kilobots)
        nearestRobotX, nearestRobotY = self._findClosest(deltaTime, kilobots)
        self._move(deltaTime, nearestRobotX, nearestRobotY)

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
        bitMapX = int(self.x/self.bitMapScalingFactor)
        bitMapY = int(self.y/self.bitMapScalingFactor)

        bitMapVal = self.bitMapArray[bitMapX, bitMapY]
        dx = self.x - nearestX
        dy = self.y - nearestY
        rVector = np.array([dx, dy, 0])
        w = np.cross(rVector, np.array([0, 0, 1]))
        tempVector = w / np.sqrt( np.dot(w, w) ) + ( preferedDistance - np.sqrt(dx**2 + dy**2) ) / ( preferedDistance - 2 * size ) * rVector / np.sqrt(np.dot(rVector,rVector))
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
        self.renderer.drawText((255, 0, 0), str(self.gradientVal), position)


class KilobotOrigin(Kilobot):
    def timestep(self, deltaTime, kilobots):
        pass

def getPositions(file='data/initPos.csv'):
    positions = []
    with open(file, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=':')
        for row in reader:
            x= int(row[0])
            y = int(row[1])
            positions.append([x,y])
    return positions


def getBitMap(file='data/bitmap.csv'):
    bitArray = []
    with open(file, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            row=[int(i) for i in row]
            bitArray.append(row)

    bitArray =  np.asarray(bitArray)
    return np.asmatrix(bitArray)



a = getBitMap()
print(a)
