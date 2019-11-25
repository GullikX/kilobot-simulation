#!/usr/bin/env python3
import pygame
import sys

from kilobot import Kilobot

windowSize = (1280, 720)
backgroundColor = (20, 20, 20)
deltaTime = 1
nKilobots = 10


def main():
    # Init pygame
    pygame.init()
    screen = pygame.display.set_mode(windowSize)
    pygame.display.set_caption("")
    running = True

    # Create kilobots
    kilobots = [None] * nKilobots
    for i in range(nKilobots):
        kilobots[i] = Kilobot(windowSize)

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
            kilobot.timestep(deltaTime)

        # Drawing
        screen.fill(backgroundColor)
        for kilobot in kilobots:
            kilobot.draw(screen)
        pygame.display.update()

    pygame.display.quit()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
