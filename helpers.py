import numpy as np
import math
def calcScalingFactor(n, bitArray, d, k=1.2,k2=0.05):

    nBits = sum(sum(i for i in bitArray))
    A3 =  k2*8*math.pi*d**2/(nBits*8)
    A1  = k*math.pi*d**2*n/(nBits*4)
    bitMapScalingFactor = math.sqrt(A3**2/4 + A1) - A3/2
    print(bitMapScalingFactor)
    return bitMapScalingFactor


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

#bitMapArray = np.loadtxt("data/bitmap.csv")
#calcScalingFactor(100, bitMapArray, 35, 1.2, 0.05)
