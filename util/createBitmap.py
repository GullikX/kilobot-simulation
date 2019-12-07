#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import sys


def main():
    if len(sys.argv)  != 3:
        print(f"Usage: {sys.argv[0]} <png file> <threshold (0.0...1.0)>")
        return
    imageFile = sys.argv[1]
    threshold = float(sys.argv[2])
    outputFile = f"{imageFile}.csv"

    image = 1 - plt.imread(imageFile)
    grayscaleImage = np.mean(image, axis=2)
    binaryBitmap = (grayscaleImage > threshold).astype(np.int_)
    np.savetxt(outputFile, binaryBitmap, fmt="%d")
    print(f"Saved output as '{outputFile}'.")


if __name__ == "__main__":
    main()
