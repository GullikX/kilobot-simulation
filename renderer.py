import numpy as np
import pygame

backgroundColor = (20, 20, 20)
bitMapColor = (50, 50, 50)
windowCaption = "Kilobot simulation"

scaleFactor = 1.25
fontSizeFactor = 25
xOffset = 640
yOffset = 360
yFlip = -1


class Renderer:
    def __init__(self, windowSize, bitMapArray, bitMapScalingFactor):
        self.screen = pygame.display.set_mode(windowSize)
        self.bitMapArray = np.flipud(bitMapArray)
        self.bitMapScalingFactor = bitMapScalingFactor
        pygame.display.set_caption(windowCaption)
        self.textFont = pygame.font.Font(pygame.font.get_default_font(), int(fontSizeFactor * scaleFactor))

    def clearScreen(self):
        self.screen.fill(backgroundColor)

    def drawBitMap(self):
        for i in range(np.shape(self.bitMapArray)[0]):
            for j in range(np.shape(self.bitMapArray)[1]):
                if self.bitMapArray[i, j] == 0:
                    continue
                coordinates = (j * self.bitMapScalingFactor, i * self.bitMapScalingFactor)
                size = (self.bitMapScalingFactor, self.bitMapScalingFactor)
                self.drawRectangle(bitMapColor, coordinates, size)


    def drawCircle(self, color, coordinates, size):
        screenPosition = [None] * 2
        screenPosition[0] = int(coordinates[0] * scaleFactor + xOffset)
        screenPosition[1] = int(coordinates[1] * scaleFactor * yFlip + yOffset)
        screenSize = int(size * scaleFactor)

        pygame.draw.circle(self.screen, color, screenPosition, screenSize)

    def drawLine(self, color, coordinatesStart, coordinatesEnd, width):
        screenPositionStart = [None] * 2
        screenPositionStart[0] = int(coordinatesStart[0] * scaleFactor + xOffset)
        screenPositionStart[1] = int(coordinatesStart[1] * scaleFactor * yFlip + yOffset)

        screenPositionEnd = [None] * 2
        screenPositionEnd[0] = int(coordinatesEnd[0] * scaleFactor + xOffset)
        screenPositionEnd[1] = int(coordinatesEnd[1] * scaleFactor * yFlip + yOffset)

        screenWidth = int(width * scaleFactor)

        pygame.draw.line(self.screen, color, screenPositionStart, screenPositionEnd, screenWidth)

    def drawRectangle(self, color, coordinates, size):
        screenPosition = [None] * 2
        screenPosition[0] = int(coordinates[0] * scaleFactor + xOffset)
        screenPosition[1] = int(coordinates[1] * scaleFactor * yFlip + yOffset)

        screenSize = [None] * 2
        screenSize[0] = int(size[0] * scaleFactor)
        screenSize[1] = int(size[1] * scaleFactor * yFlip)

        rect = pygame.Rect(
            screenPosition[0], screenPosition[1],
            screenSize[0], screenSize[1],
        )

        pygame.draw.rect(self.screen, color, rect)

    def drawText(self, color, string, coordinates):
        screenPosition = [None] * 2
        screenPosition[0] = int(coordinates[0] * scaleFactor + xOffset)
        screenPosition[1] = int(coordinates[1] * scaleFactor * yFlip + yOffset)

        text = self.textFont.render(string, True, color)
        self.screen.blit(text, (
            screenPosition[0] - text.get_width() // 2,
            screenPosition[1] - text.get_height() // 2,
            )
        )

    def updateDisplay(self):
        pygame.display.update()
        return self.screen
