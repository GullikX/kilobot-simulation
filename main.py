#!/usr/bin/env python3
import datetime
import numpy as np
import random
import sys

from kilobot import Kilobot, KilobotOrigin
from renderer import Renderer
#from rendererdummy import Renderer
from helpers import generateBotCoords, calcScalingFactor
deltaTime = 1
nKilobotsOrigin = 4
nrOfRobots = 20
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

    # Create graphical window
    renderer = Renderer(bitMapArray, bitMapScalingFactor)

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

    # Run simulation
    iTimestep = 0
    try:
        while renderer.running:
            renderer.handleEvents()

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

            iTimestep += 1
    except KeyboardInterrupt:
        pass

    renderer.quit()

    # Calculate score
    score = 0
    for kilobot in kilobots:
        score += kilobot.calcOverlappingA()
    figArea = np.sum(bitMapArray)*bitMapScalingFactor**2
    score = score/figArea
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

if __name__ == "__main__":
    main()
