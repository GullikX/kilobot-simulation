#!/usr/bin/env python3
import datetime
import numpy as np
import random
import sys

from kilobot import Kilobot, KilobotOrigin, State
#from renderer import Renderer
from rendererdummy import Renderer
from helpers import generateBotCoords, calcScalingFactor
deltaTime = 1
nKilobotsOrigin = 4
nrOfRobots = 100
initialPositionsFile = "data/initPos.csv"
bitMapFile = "data/bitmap.csv"
stopCriteriaCheckInterval = 10000  # timesteps

def main(nrOfBots, initialPositionsFile, bitMapFile, saveState=False):
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
            kilobots[iKilobot] = KilobotOrigin(renderer, initialPositions[iKilobot], startAngle, bitMapArray, bitMapScalingFactor, gradientVal, nrOfRobots)
        else:
            startAngle = random.uniform(0, 2*np.pi)
            gradientVal = np.inf
            kilobots[iKilobot] = Kilobot(renderer, initialPositions[iKilobot], startAngle, bitMapArray, bitMapScalingFactor, gradientVal, nrOfRobots)
    nRobotsPerStatePrev = [0, 0, nKilobotsOrigin]

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

            # Stop simulation if no new bots have joined the shape in a long time
            if iTimestep % stopCriteriaCheckInterval == 0:
                nRobotsPerState = [0, 0, 0]
                for kilobot in kilobots:
                    nRobotsPerState[kilobot.state] += 1
                print(f"[Timestep {iTimestep}] Number of robots per state: {nRobotsPerState}")
                if (nRobotsPerState[State.JOINED_SHAPE] > nKilobotsOrigin and
                        nRobotsPerState == nRobotsPerStatePrev):
                    print("No robots have changed state since last check. Exiting...")
                    renderer.running = False
                nRobotsPerStatePrev = nRobotsPerState

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

    if saveState == True:
    # Write state to csv files
        positions = np.zeros((nrOfRobots, 3))
        states = np.zeros((nrOfRobots, 1))
        for iBot in range(nrOfRobots):
            bot = kilobots[iBot]
            positions[iBot, 0] = bot.pActual[0]
            positions[iBot, 1] = bot.pActual[1]
            positions[iBot, 2] = bot.direction
            states[iBot, 0] = int(bot.state)
        np.savetxt(f"{date_time}-positions.csv", positions)
        np.savetxt(f"{date_time}-states.csv", states, fmt="%d")

    kilobots[0].resetSpatialMap()

    return score


listOfNrOfRobots = [10, 20, 50, 100, 150, 300, 500, 700, 800, 900, 1000]
for i in range(len(listOfNrOfRobots)):
    nrOfRobots = listOfNrOfRobots[i]
    data = {}
    score = 0
    for n in range(5):
        score += main(nrOfRobots, initialPositionsFile, bitMapFile)
    data = {str(nrOfRobots), score/5}

if __name__ == "__main__":
    main()
