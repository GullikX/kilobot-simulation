class Renderer:
    def __init__(self, bitMapArray, bitMapScalingFactor):
        self.running = True

    def handleEvents(self):
        pass

    def clearScreen(self):
        pass

    def drawBitMap(self):
        pass

    def drawCircle(self, color, coordinates, size, width=0):
        pass

    def drawLine(self, color, coordinatesStart, coordinatesEnd, width):
        pass

    def drawText(self, color, string, coordinates):
        pass

    def updateDisplay(self):
        pass

    def quit(self):
        pass
