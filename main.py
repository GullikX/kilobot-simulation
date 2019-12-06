#!/usr/bin/env python3
import pygame
import numpy as np
import random
import sys

from kilobot import Kilobot, KilobotOrigin, getPositions
from renderer import Renderer

windowSize = (1280, 720)
deltaTime = 1
nKilobots = 10
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
            startAngle = random.uniform(0, 2*np.pi)
            kilobots[iKilobot] = Kilobot(renderer, kilobotOriginPositions[iKilobot], startAngle)

    while running:
        # Input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

        # Logic
        #for kilobot in kilobots:
        #    kilobot.timestep(deltaTime, kilobots)

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
