import pygame
import numpy as np
import random

color = (128, 128, 128)
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
        pygame.draw.circle(screen, color, (int(self.x), int(self.y)), size)
