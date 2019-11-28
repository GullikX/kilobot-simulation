import numpy as np

colorBody = (192, 192, 192)
colorDirectionLine = (25, 118, 210)
size = 30
velocity = 1


class Kilobot:
    counter = 0;
    def __init__ (self, renderer, startPosition, startDirection):
        self.renderer = renderer
        self.x, self.y = startPosition
        self.direction = startDirection
        self.gradientVal = 1    #See paper
        Kilobot.counter += 1

    def _setGradient(self, kilobots, gDist):
        highestGradient = 0;
        for bot in kilobots:
            xDiff = self.x - bot.x
            yDiff = self.y - bot.y
            dist = sqrt(xDiff^2 + yDiff^2)

            if dist < gDist and bot.gradientVal > highestGradient:
                highestGradient = bot.gradientVal
        self.gradientVal = 1 + highestGradient


    def timestep(self, deltaTime):
        self.x += velocity * deltaTime * np.cos(self.direction)
        self.y += velocity * deltaTime * np.sin(self.direction)

    def draw(self):
        position = (int(self.x), int(self.y))
        directionLineTarget = (
                int(self.x + np.cos(self.direction) * size),
                int(self.y + np.sin(self.direction) * size),
        )
        self.renderer.drawCircle(colorBody, position, size)
        self.renderer.drawLine(colorDirectionLine, position, directionLineTarget, size/5)


class KilobotOrigin(Kilobot):
    def timestep(self, deltaTime):
        pass
