import numpy as np

colorBody = (192, 192, 192)
colorDirectionLine = (25, 118, 210)
size =15
velocity = 1
turnSpeed = np.pi / 36
communicationRange = 60

preferedDistance = 50 #Bugged for <= 2 * size
maxAngleError = np.pi / 36

class Kilobot:
    def __init__ (self, renderer, startPosition, startDirection, bitMapArray, bitMapScalingFactor, gradientVal):
        self.renderer = renderer
        self.x, self.y = startPosition
        self.direction = startDirection
        self.bitMapArray = bitMapArray
        self.bitMapScalingFactor = bitMapScalingFactor
        self.gradientVal = gradientVal

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
        bitMapVal = self._getBinaryCoord()
        if bitMapVal == 0:  #move into position
            self._move(deltaTime, nearestRobotX, nearestRobotY)
            bitMapVal = self._getBinaryCoord()
            while bitMapVal == 1:      #move until we leave bitShape
                self._move(deltaTime, nearestRobotX, nearestRobotY)
                bitMapVal = self._getBinaryCoord()

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

    def _getBinaryCoord(self):
        bitMapVal = 0
        if self.x >= 0 and  self.y >= 0:
            bitMapX = int(self.x/self.bitMapScalingFactor)
            bitMapY = int(self.y/self.bitMapScalingFactor)

            bitShape = self.bitMapArray.shape
            if bitMapX >= 0 and bitMapX < bitShape[0] and bitMapY >= 0 and bitMapY < bitShape[1]:
                print(bitMapX, bitMapY)
                bitMapVal = self.bitMapArray[bitMapX, bitMapY]
        return bitMapVal


class KilobotOrigin(Kilobot):
    def timestep(self, deltaTime, kilobots):
        pass
