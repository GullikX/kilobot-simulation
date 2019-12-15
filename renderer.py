import numpy as np
import pygame

backgroundColor = (20, 20, 20)
bitMapColor = (50, 50, 50)
windowCaption = "Kilobot simulation"

windowSize = (1280, 720)
fps = 60
scaleFactor = 1.2
fontSizeFactor = 15
xOffset = 640
yOffset = 360
yFlip = -1
zoomInFactor = 1.1
zoomOutFactor = 0.9


class Renderer:
    def __init__(self, bitMapArray, bitMapScalingFactor):
        self.bitMapArray = np.flipud(bitMapArray)
        self.bitMapScalingFactor = bitMapScalingFactor
        self.scaleFactor = scaleFactor
        self.xOffset = xOffset
        self.yOffset = yOffset
        self.dragging = False
        self.dragStartPos = (0, 0)

        pygame.init()
        self.screen = pygame.display.set_mode(windowSize)
        pygame.display.set_caption(windowCaption)
        self.fpsClock = pygame.time.Clock()
        self.running = True

    def handleEvents(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    self.dragStart(event)
                if event.button == 4:
                    self.zoomIn()
                elif event.button == 5:
                    self.zoomOut()
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    self.dragStop()
            elif event.type == pygame.MOUSEMOTION:
                self.drag(event)

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


    def drawCircle(self, color, coordinates, size, width=0):
        screenPosition = [None] * 2
        screenPosition[0] = int(coordinates[0] * self.scaleFactor + self.xOffset)
        screenPosition[1] = int(coordinates[1] * self.scaleFactor * yFlip + self.yOffset)
        screenSize = int(size * self.scaleFactor)

        pygame.draw.circle(self.screen, color, screenPosition, screenSize, width)

    def drawLine(self, color, coordinatesStart, coordinatesEnd, width):
        screenPositionStart = [None] * 2
        screenPositionStart[0] = int(coordinatesStart[0] * self.scaleFactor + self.xOffset)
        screenPositionStart[1] = int(coordinatesStart[1] * self.scaleFactor * yFlip + self.yOffset)

        screenPositionEnd = [None] * 2
        screenPositionEnd[0] = int(coordinatesEnd[0] * self.scaleFactor + self.xOffset)
        screenPositionEnd[1] = int(coordinatesEnd[1] * self.scaleFactor * yFlip + self.yOffset)

        screenWidth = int(width * self.scaleFactor)

        pygame.draw.line(self.screen, color, screenPositionStart, screenPositionEnd, screenWidth)

    def drawRectangle(self, color, coordinates, size):
        screenPosition = [None] * 2
        screenPosition[0] = int(coordinates[0] * self.scaleFactor + self.xOffset)
        screenPosition[1] = int(coordinates[1] * self.scaleFactor * yFlip + self.yOffset)

        screenSize = [None] * 2
        screenSize[0] = int(size[0] * self.scaleFactor)
        screenSize[1] = int(size[1] * self.scaleFactor * yFlip)

        rect = pygame.Rect(
            screenPosition[0], screenPosition[1],
            screenSize[0], screenSize[1],
        )

        pygame.draw.rect(self.screen, color, rect)

    def drawText(self, color, string, coordinates):
        screenPosition = [None] * 2
        screenPosition[0] = int(coordinates[0] * self.scaleFactor + self.xOffset)
        screenPosition[1] = int(coordinates[1] * self.scaleFactor * yFlip + self.yOffset)
        textFont = pygame.font.Font(pygame.font.get_default_font(), int(fontSizeFactor * self.scaleFactor))

        textFont = pygame.font.Font(pygame.font.get_default_font(), int(fontSizeFactor * self.scaleFactor))
        text = textFont.render(string, True, color)
        self.screen.blit(text, (
            screenPosition[0] - text.get_width() // 2,
            screenPosition[1] - text.get_height() // 2,
            )
        )

    def updateDisplay(self):
        pygame.display.update()
        self.fpsClock.tick(fps)
        #print(f"FPS: {self.fpsClock.get_fps()}")

    def quit(self):
        pygame.display.quit()
        pygame.quit()

    def zoomIn(self):
        self.scaleFactor *= zoomInFactor

    def zoomOut(self):
        self.scaleFactor *= zoomOutFactor

    def dragStart(self, event):
        self.dragging = True
        self.dragStartPos = (self.xOffset - event.pos[0], self.yOffset - event.pos[1])

    def drag(self, event):
        if self.dragging:
            self.xOffset = self.dragStartPos[0] + event.pos[0]
            self.yOffset = self.dragStartPos[1] + event.pos[1]

    def dragStop(self):
        self.dragging = False
