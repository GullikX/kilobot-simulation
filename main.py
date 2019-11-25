#!/usr/bin/env python3
import pygame

from kilobot import Kilobot

windowSize = (1280, 720)
backgroundColor = (0, 0, 0)
deltaTime = 1
nKilobots = 10


def main():
    # Init pygame
    pygame.init()
    screen = pygame.display.set_mode(windowSize)
    pygame.display.set_caption("")

    # Create kilobots
    kilobots = [None] * nKilobots
    for i in range(nKilobots):
        kilobots[i] = Kilobot(windowSize)

    while True:
        # Logic
        for kilobot in kilobots:
            kilobot.timestep(deltaTime)

        # Drawing
        screen.fill(backgroundColor)
        for kilobot in kilobots:
            kilobot.draw(screen)
        pygame.display.update()

if __name__ == "__main__":
    main()
