#!/usr/bin/env python3
import datetime
import pygame
import numpy as np
import random
import sys

from kilobot import Kilobot, KilobotOrigin
from renderer import Renderer
from helpers import generateBotCoords, calcScalingFactor
windowSize = (1280, 720)
fps = 60
deltaTime = 1
nKilobotsOrigin = 4
nrOfRobots = 100
initialPositionsFile = "data/initPos.csv"
bitMapFile = "data/bitmap.csv"
def main():
    # Get current time
    date_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")

    # Load data arrays
    bitMapArray = np.loadtxt(bitMapFile)
    bitMapScalingFactor = calcScalingFactor(nrOfRobots, bitMapArray,31)
#    initialPositions = np.loadtxt(initialPositionsFile)
    initialPositions = generateBotCoords(nrOfRobots, 35)
    # Init pygame
    pygame.init()
    fpsClock = pygame.time.Clock()
    renderer = Renderer(windowSize, bitMapArray, bitMapScalingFactor)
    running = True

    # Create kilobots
    nKilobots = np.shape(initialPositions)[0]
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

    iTimestep = 0
    while running:
        # Input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    renderer.dragStart(event)
                if event.button == 4:
                    renderer.zoomIn()
                elif event.button == 5:
                    renderer.zoomOut()
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    renderer.dragStop()
            elif event.type == pygame.MOUSEMOTION:
                renderer.drag(event)

        # Logic
        random.shuffle(kilobots)
        for kilobot in kilobots:
            kilobot.timestep(iTimestep, deltaTime, kilobots)

        # Drawing
        renderer.clearScreen()
        renderer.drawBitMap()
        for kilobot in kilobots:
            kilobot.draw()
        renderer.updateDisplay()

        # Wait until next frame
        fpsClock.tick(fps)
        #print(f"FPS: {fpsClock.get_fps()}")
        iTimestep += 1

    # Calculate score
    score = 0
    for kilobot in kilobots:
        score += kilobot._calculateScore() / nrOfRobots
    print(f"Score: {score}")

    # Write state to csv files
    positions = np.zeros((nrOfRobots, 2))
    states = np.zeros((nrOfRobots, 1))
    for iBot in range(nrOfRobots):
        bot = kilobots[iBot]
        positions[iBot, 0] = bot.pos[0]
        positions[iBot, 1] = bot.pos[1]
        states[iBot, 0] = int(bot.state)
    np.savetxt(f"{date_time}-positions.csv", positions)
    np.savetxt(f"{date_time}-states.csv", states, fmt="%d")

    pygame.display.quit()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
