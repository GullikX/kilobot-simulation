#!/usr/bin/env python3
import numpy as np
import pygame
import sys

from helpers import calcScalingFactor, calcOverlappingA
from renderer import Renderer

colorRobotBody = (
    (229, 57, 53),   # Waiting
    (66, 175, 80),   # Moving
    (25, 118, 210),  # Joined shape
)
colorRobotDirectionLine = (192, 192, 192)
robotSize = 15
nScorePoints = 100


def calculateScore(positions, bitMapArray, bitMapScalingFactor):
    nRobots = np.shape(positions)[0]
    score = 0
    for iPosition in range(nRobots):
        position = positions[iPosition, 0:2]
        score += calcOverlappingA(position, robotSize, nScorePoints, bitMapArray, bitMapScalingFactor)
        print(score)
    figArea = np.sum(bitMapArray)*bitMapScalingFactor**2
    score = score/figArea
    return score


def drawBot(renderer, position, direction, state):
    colorBody = colorRobotBody[state]

    directionLineTarget = (
        int(position[0] + np.cos(direction) * robotSize),
        int(position[1] + np.sin(direction) * robotSize),
    )

    renderer.drawCircle(colorBody, position, robotSize)
    renderer.drawLine(colorRobotDirectionLine, position, directionLineTarget, robotSize/4)


def main():
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <positions file> <states file> <bitmap file>")
        return

    positionsFile = sys.argv[1]
    statesFile = sys.argv[2]
    bitMapFile = sys.argv[3]

    positions = np.loadtxt(positionsFile)  # x, y, direction
    states = np.loadtxt(statesFile, dtype=int)
    nBots = len(states)
    bitMapArray = np.loadtxt(bitMapFile)
    bitMapScalingFactor = calcScalingFactor(nBots, bitMapArray, 31)

    score = calculateScore(positions, bitMapArray, bitMapScalingFactor)
    print(f"Score: {score}")

    renderer = Renderer(bitMapArray, bitMapScalingFactor)

    try:
        while renderer.running:
            renderer.handleEvents()
            renderer.clearScreen()
            renderer.drawBitMap()
            for iBot in range(nBots):
                position = (positions[iBot, 0], positions[iBot, 1])
                direction = positions[iBot, 2]
                state = states[iBot]
                drawBot(renderer, position, direction, state)
            renderer.updateDisplay()
    except KeyboardInterrupt:
        pass

    renderer.quit()


if __name__ == "__main__":
    main()
