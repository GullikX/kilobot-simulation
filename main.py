#!/usr/bin/env python3
import pygame

windowSize = (1280, 720)
backgroundColor = (0, 0, 0)
robotColor = (128, 128, 128)
robotSize = 30


def main():
    pygame.init()
    screen = pygame.display.set_mode(windowSize)
    pygame.display.set_caption("")
    i = 0

    while True:
        i = (i + 1) % windowSize[0]
        robotPosition = (i, 100)

        screen.fill(backgroundColor)
        pygame.draw.circle(screen, robotColor, robotPosition, robotSize)
        pygame.display.update()

if __name__ == "__main__":
    main()
