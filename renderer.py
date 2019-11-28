import pygame

backgroundColor = (20, 20, 20)
windowCaption = "Kilobot simulation"

xScale = 1
yScale = 1
xOffset = 0
yOffset = 0

class Renderer:
    def __init__(self, windowSize):
        self. screen = pygame.display.set_mode(windowSize)
        pygame.display.set_caption(windowCaption)

    def clearScreen(self):
        self.screen.fill(backgroundColor)

    def drawCircle(self, color, coordinates, size):
        screenPosition = [None] * 2
        screenPosition[0] = int((coordinates[0] + xOffset) * xScale)
        screenPosition[1] = int((coordinates[1] + yOffset) * yScale)

        pygame.draw.circle(self.screen, color, screenPosition, size)

    def drawLine(self, color, coordinatesStart, coordinatesEnd, width):
        screenPositionStart = [None] * 2
        screenPositionStart[0] = int((coordinatesStart[0] + xOffset) * xScale)
        screenPositionStart[1] = int((coordinatesStart[1] + yOffset) * yScale)

        screenPositionEnd = [None] * 2
        screenPositionEnd[0] = int((coordinatesEnd[0] + xOffset) * xScale)
        screenPositionEnd[1] = int((coordinatesEnd[1] + yOffset) * yScale)

        pygame.draw.line(self.screen, color, screenPositionStart, screenPositionEnd, int(width))

    def updateDisplay(self):
        pygame.display.update()
