import pygame

backgroundColor = (20, 20, 20)
windowCaption = "Kilobot simulation"

scaleFactor = 0.8
xOffset = 320
yOffset = 180

class Renderer:
    def __init__(self, windowSize):
        self. screen = pygame.display.set_mode(windowSize)
        pygame.display.set_caption(windowCaption)

    def clearScreen(self):
        self.screen.fill(backgroundColor)

    def drawCircle(self, color, coordinates, size):
        screenPosition = [None] * 2
        screenPosition[0] = int(coordinates[0] * scaleFactor + xOffset)
        screenPosition[1] = int(coordinates[1] * scaleFactor + yOffset)
        screenSize = int(size * scaleFactor)

        pygame.draw.circle(self.screen, color, screenPosition, screenSize)

    def drawLine(self, color, coordinatesStart, coordinatesEnd, width):
        screenPositionStart = [None] * 2
        screenPositionStart[0] = int(coordinatesStart[0] * scaleFactor + xOffset)
        screenPositionStart[1] = int(coordinatesStart[1] * scaleFactor + yOffset)

        screenPositionEnd = [None] * 2
        screenPositionEnd[0] = int(coordinatesEnd[0] * scaleFactor + xOffset)
        screenPositionEnd[1] = int(coordinatesEnd[1] * scaleFactor + yOffset)

        screenWidth = int(width * scaleFactor)

        pygame.draw.line(self.screen, color, screenPositionStart, screenPositionEnd, screenWidth)

    def updateDisplay(self):
        pygame.display.update()
        return self.screen
