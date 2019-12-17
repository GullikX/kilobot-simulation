import numpy as np
import math


def calcScalingFactor(n, bitArray, d, k=1.2,k2=0.05):
    nBits = sum(sum(i for i in bitArray))
    A3 =  k2*8*math.pi*d**2/(nBits*8)
    A1  = k*math.pi*d**2*n/(nBits*4)
    bitMapScalingFactor = math.sqrt(A3**2/4 + A1) - A3/2
    return bitMapScalingFactor


def isInsideShape(bitMapArray, scalingFactor, pos):
    dim = np.multiply(bitMapArray.shape, scalingFactor)
    if np.all(pos >= 0) and np.all(pos < dim):
        p = pos/scalingFactor
        bitMapVal = bitMapArray[p[0].astype(int), p[1].astype(int)]
        return bool(bitMapVal)
    return False


def generateBotCoords(nrOfBots,preferedDistance):
    fourInitial = [[-1/2,0],[0, -np.sqrt(3)/2],
                    [1/2,0], [0, np.sqrt(3)/2]]
    p = np.array([1/2, -np.sqrt(3)])
    k = int(nrOfBots**(2/3))
    for i in range(4,nrOfBots):
        j = i - 4
        a = int(j/k)
        b = int(j%k)
        currentPos = p + np.array([1,0])*b + np.array([1/2, -np.sqrt(3)/2])*a
        currentPos = np.ndarray.tolist(currentPos)
        fourInitial.append(currentPos)
    listOfBots = np.asarray([fourInitial])
    listOfBots = listOfBots*preferedDistance
    listOfBots = np.asmatrix(listOfBots)
    return listOfBots

def calcOverlappingA(circlePosition, circleRadius, nParamPoints, bitMapArray, bitMapScalingFactor):
    theta = np.linspace(0,2*np.pi, nParamPoints)
    x = circlePosition[0] + circleRadius * np.cos(theta)
    y = circlePosition[1] + circleRadius * np.sin(theta)
    p = []
    pos = np.array([x[0],y[0]]).astype(int)
    boolP = isInsideShape(bitMapArray, bitMapScalingFactor, pos)
    for i in range(nParamPoints):
        pos = np.array([x[i],y[i]]).astype(int)
        bitMapVal = isInsideShape(bitMapArray, bitMapScalingFactor, pos)
        if len(p) < 2:
            if boolP and not bool(bitMapVal):
                p.append(i)
                boolP = False
            elif not boolP and bool(bitMapVal):
                p.append(i)
                boolP = True

    if len(p) == 0:
        if boolP:
            return np.pi*circleRadius**2
        else:
            return -np.pi*circleRadius**2

    centralAngle = theta[p[1]] -  theta[p[0]]
    partialSquareArea = circleRadius**2/2*(centralAngle - (np.sin(centralAngle)))
    if boolP:
        partialSquareArea = np.pi*circleRadius**2 - partialSquareArea
    return (2*partialSquareArea) - (np.pi*circleRadius**2)
