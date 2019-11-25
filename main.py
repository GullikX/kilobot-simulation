#!/usr/bin/env python3
import pygame

windowSize = (1280, 720)


def main():
    pygame.init()
    screen = pygame.display.set_mode(windowSize)
    pygame.display.set_caption("")
    screen.fill((0, 0, 0))

    i = 0

    while True:
        i = (i + 1) % windowSize[0]
        screen.fill((0, 0, 0))
        pygame.draw.circle(screen, (255, 255, 255), (i, 100), 30)
        pygame.display.update()

if __name__ == "__main__":
    main()
