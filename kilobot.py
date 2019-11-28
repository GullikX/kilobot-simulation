import pygame
import numpy as np
import random

colorBody = (192, 192, 192)
colorDirectionLine = (25, 118, 210)
size = 30
velocity = 1


class Kilobot:
    counter = 0;
    def __init__ (self, windowSize, coordInit = False):
        self.x = random.uniform(0, windowSize[0])
        self.y = random.uniform(0, windowSize[1])
        self.direction = random.uniform(0, 2*np.pi)
        self.gradientVal = 1    %See paper
        Kilobot.counter += 1

    def _setGradient(self, kilobots, gDist):
        highestGradient = 0;
        for bot in kilobots:
            xDiff = self.x - bot.x
            yDiff = self.y - bot.y
            dist = sqrt(xDiff^2 + yDiff^2)

            if dist < gDist && bot.gradientVal > highestGradient:
                highestGradient = bot.gradientVal
        self.gradientVal = 1 + highestGradient


    def timestep(self, deltaTime):
        self.x += velocity * deltaTime * np.cos(self.direction)
        self.y += velocity * deltaTime * np.sin(self.direction)

    def draw(self, screen):
        position = (int(self.x), int(self.y))
        directionLineTarget = (
                int(self.x + np.cos(self.direction) * size),
                int(self.y + np.sin(self.direction) * size),
        )
        pygame.draw.circle(screen, colorBody, position, size)
        pygame.draw.line(screen, colorDirectionLine, position, directionLineTarget, int(size/5))


class KilobotOrigin(Kilobot):
    def __init__ (self, position):
        self.x = position[0]
        self.y = position[1]
        self.direction = random.uniform(0, 2*np.pi)

    def timestep(self, deltaTime):
        pass
