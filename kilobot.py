import numpy as np

colorBody = (192, 192, 192)
colorDirectionLine = (25, 118, 210)
size = 30
velocity = 1
turnSpeed = np.pi / 36

preferedDistance = 50 #Bugged for <= 2 * size
maxAngleError = np.pi / 36

class Kilobot:
    counter = 0;
    def __init__ (self, renderer, startPosition, startDirection):
        self.renderer = renderer
        self.x, self.y = startPosition
        self.direction = startDirection
        self.gradientVal = 1    #See paper
        Kilobot.counter += 1

    def _setGradient(self, kilobots, gDist):
        highestGradient = 0
        for bot in kilobots:
            xDiff = self.x - bot.x
            yDiff = self.y - bot.y
            dist = sqrt(xDiff^2 + yDiff^2)

            if dist < gDist and bot.gradientVal > highestGradient:
                highestGradient = bot.gradientVal
        self.gradientVal = 1 + highestGradient


    def timestep(self, deltaTime):
        self.x += velocity * deltaTime * np.cos(self.direction)
        self.y += velocity * deltaTime * np.sin(self.direction)

    def findClosest(self, deltaTime, kilobots):
        rmax = 100
        nearestX = 1
        nearestY = 1

        for bot in kilobots:
            r = np.sqrt( (self.x - bot.x)^2 + (self.y - bot.y)^2 )
            if r < rmax:
                rmax = r
                nearestX = bot.x
                nearestY = bot.y

        kilobot.move(self, deltaTime, nearestX, nearestY)

    def move(self, deltaTime, nearestX, nearestY):
        dx = self.x - nearestX
        dy = self.y - nearestY
        rVector = np.array([dx, dy, 0])
        w = np.cross(rVector, np.array([0, 0, 1]))
        tempVector = w / np.sqrt( np.dot(w, w) ) + ( preferedDistance - np.sqrt(dx^2 + dy^2) ) / ( preferedDistance - 2 * size ) * rVector / np.sqrt(rVector,rVector)
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


class KilobotOrigin(Kilobot):
    def timestep(self, deltaTime):
        pass

def getPositions(file='initPos.csv'):
    positions = []
    with open(file, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=':')
        for row in reader:
            x= int(row[0])
            y = int(row[1])
            positions.append([x,y])
    return positions
