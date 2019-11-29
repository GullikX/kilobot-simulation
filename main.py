#!/usr/bin/env python3
import pygame
import numpy as np
import random
import sys

from kilobot import Kilobot, KilobotOrigin, getPositions
from renderer import Renderer

windowSize = (640, 360)
deltaTime = 1
nKilobots = 5
nKilobotsOrigin = 4

kilobotOriginPositions = getPositions()

def main():
    # Init pygame
    pygame.init()
    renderer = Renderer(windowSize)
    running = True

    # Create kilobots
    kilobots = [None] * nKilobots
    for iKilobot in range(nKilobots):
        if iKilobot < nKilobotsOrigin:
            kilobots[iKilobot] = KilobotOrigin(renderer, kilobotOriginPositions[iKilobot], 0)
        else:
            startPosition = (  # Temporary, use proper start positions and angles later
                random.uniform(-windowSize[0]/2, windowSize[0]/2),
                random.uniform(-windowSize[1]/2, windowSize[1]/2),
            )
            startAngle = random.uniform(0, 2*np.pi)
            kilobots[iKilobot] = Kilobot(renderer, startPosition, startAngle)

    while running:
        # Input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

        # Logic
        for kilobot in kilobots:
            kilobot.findClosest(deltaTime,kilobots)

        # Drawing
        renderer.clearScreen()
        for kilobot in kilobots:
            kilobot.draw()
        surf = renderer.updateDisplay()


    pygame.display.quit()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
