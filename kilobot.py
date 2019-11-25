import pygame
import numpy as np
import random

colorBody = (192, 192, 192)
colorDirectionLine = (25, 118, 210)
size = 30
velocity = 1


class Kilobot:
    def __init__ (self, windowSize, coordInit = False):
        self.x = random.uniform(0, windowSize[0])
        self.y = random.uniform(0, windowSize[1])
        self.direction = random.uniform(0, 2*np.pi)

        #if coordInit = True:
        #    #TODO initialize the pos of the fours coordinate robots

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
