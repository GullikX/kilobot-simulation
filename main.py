#!/usr/bin/env python3
import pygame
import numpy as np
import random
import sys

from kilobot import Kilobot, KilobotOrigin
from renderer import Renderer

windowSize = (1280, 720)
fps = 60
deltaTime = 1
nKilobots = 10
nKilobotsOrigin = 4
bitMapFile = "data/bitmap.csv"
bitMapScalingFactor = 100
initialPositionsFile = "data/initPos.csv"

def main():
    # Load data arrays
    bitMapArray = np.loadtxt(bitMapFile)
    initialPositions = np.loadtxt(initialPositionsFile)

    # Init pygame
    pygame.init()
    fpsClock = pygame.time.Clock()
    renderer = Renderer(windowSize, bitMapArray, bitMapScalingFactor)
    running = True

    # Create kilobots
    kilobots = [None] * nKilobots
    for iKilobot in range(nKilobots):
        if iKilobot < nKilobotsOrigin:
            startAngle = 0
            if iKilobot == 0:
                gradientVal = 0
            else:
                gradientVal = 1
            kilobots[iKilobot] = KilobotOrigin(renderer, initialPositions[iKilobot], startAngle, bitMapArray, bitMapScalingFactor, gradientVal)
        else:
            startAngle = random.uniform(0, 2*np.pi)
            gradientVal = np.inf
            kilobots[iKilobot] = Kilobot(renderer, initialPositions[iKilobot], startAngle, bitMapArray, bitMapScalingFactor, gradientVal)

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
            kilobot.timestep(deltaTime, kilobots)

        # Drawing
        renderer.clearScreen()
        renderer.drawBitMap()
        for kilobot in kilobots:
            kilobot.draw()
        surf = renderer.updateDisplay()

        # Wait until next frame
        fpsClock.tick(fps)

    pygame.display.quit()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
