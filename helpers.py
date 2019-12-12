import numpy as np
import math
def calcScalingFactor(n, bitArray, d, k,k2):

    nBits = sum(sum(i for i in bitArray))
    A3 =  k2*8*math.pi*d**2/(nBits*8)
    A1  = k*math.pi*d**2*n/(nBits*4)
    bitMapScalingFactor = math.sqrt(A3**2/4 + A1) - A3/2
    print(bitMapScalingFactor)
    return bitMapScalingFactor


bitMapArray = np.loadtxt("data/bitmap.csv")
calcScalingFactor(100, bitMapArray, 35, 1.2, 0.05)
