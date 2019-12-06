import pygame
import pygame.freetype

backgroundColor = (20, 20, 20)
windowCaption = "Kilobot simulation"

scaleFactor = 1.25
fontSizeFactor = 25
xOffset = 640
yOffset = 360
yFlip = -1


class Renderer:
    def __init__(self, windowSize):
        self.screen = pygame.display.set_mode(windowSize)
        pygame.display.set_caption(windowCaption)
        self.textFont = pygame.font.Font(pygame.font.get_default_font(), int(25 * scaleFactor))

    def clearScreen(self):
        self.screen.fill(backgroundColor)

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
