import numpy as np
from queue import PriorityQueue
import csv

colorBody = (192, 192, 192)
colorDirectionLine = (25, 118, 210)
size = 30
velocity = 1


class Kilobot:
    counter = 0;
    q = PriorityQueue()
    def __init__ (self, renderer, startPosition, startDirection):
        self.renderer = renderer
        self.x, self.y = startPosition
        self.direction = startDirection
        self.gradientVal = 1    #See paper

        Kilobot.counter += 1
        Kilobot.q.put(self) #Har inte testat ifall detta funkar

    def _setGradient(self, kilobots, gDist):
        highestGradient = 0;
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
