import numpy as np
import numpy.linalg


class SpatialMap:
    def __init__(self, stepSize, nCells):
        self.stepSize = stepSize
        self.grid = [[None]*nCells for i in range(nCells)]

    def addEntry(self, obj, pos):
        try:
            i, j = self._getIndices(pos)
            if self.grid[i][j] == None:
                self.grid[i][j] = obj
            else:
                raise ValueError("Tried to add to already occupied spot!")
        except IndexError:
            raise ValueError("Spatial map to small!")

    def removeEntry(self, obj, pos):
        try:
            i, j = self._getIndices(pos)
            if self.grid[i][j] is None:
                raise ValueError("Tried to remove None from spatial map!")
            elif self.grid[i][j] is not obj:
                raise ValueError("Tried to remove wrong object from spatial map!")
            self.grid[i][j] = None
        except IndexError:
            raise ValueError("Spatial map to small!")

    def getNeighbors(self, caller, pos, maxDistance):
        try:
            neighbors = []
            i, j = self._getIndices(pos)
            iDist = int(np.ceil(maxDistance / self.stepSize))
            for iOther in range(i - iDist, i + iDist):
                for jOther in range(j - iDist, j + iDist):
                    if self.grid[iOther][jOther] is not None:
                        obj = self.grid[iOther][jOther]
                        if obj is not caller and np.linalg.norm(obj.pActual - pos) < maxDistance:
                            neighbors.append(self.grid[iOther][jOther])
            return neighbors
        except IndexError:
            raise ValueError("Spatial map to small!")

    def _getIndices(self, pos):
        extends = len(self.grid) * self.stepSize / 2
        x = pos[0] + extends
        y = pos[1] + extends

        i = int(x / self.stepSize)
        j = int(y / self.stepSize)

        return i, j
