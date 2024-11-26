import numpy as np
import matplotlib.pyplot as plt

from scipy.spatial import Delaunay

class Blocks:
    def __init__(self, numBlocks, domainMin, domainMax):

        self.dx = numBlocks/(domainMax - domainMin)
        self.numBlocks = numBlocks
        self.domainMin = domainMin
        self.domainMax = domainMax
        self.totalNumBlocks = numBlocks**3
        
        self.centers = np.zeros((self.totalNumBlocks, 3))
        iCount = 0
        for iCell in range(numBlocks):
            for jCell in range(numBlocks):
                for kCell in range(numBlocks):
        
                    self.centers[iCell*numBlocks**2 + jCell*numBlocks + kCell] = [domainMin + (0.5 + iCell) * self.dx, domainMin + (0.5 + jCell) * self.dx, domainMin + (0.5 + kCell) * self.dx] 
                    self.index = iCount
                    iCount += 1
                
                
def findHostCell(queryPointPosition, domainBlocks, domainMin, domainMax):

    for i in range(3):
        if(queryPointPosition[i] > domainMax or queryPointPosition[i] < domainMin):
            return -1

    blockXIndex = np.floor((queryPointPosition[0] - domainMin) / domainBlocks.dx)
    blockYIndex = np.floor((queryPointPosition[1] - domainMin) / domainBlocks.dx)
    blockZIndex = np.floor((queryPointPosition[2] - domainMin) / domainBlocks.dx)
        
    return 0




numBlocks =  16
domainMin = -1.0
domainMax =  1.0

# Create a Blocks object
blocks = Blocks(numBlocks, domainMin, domainMax)

quit()
num_points = 10
x = np.random.uniform(-1, 1, num_points)
y = np.random.uniform(-1, 1, num_points)
z = np.random.uniform(-1, 1, num_points)

pointCloud    = np.column_stack((x, y, z))
triangulation = Delaunay(pointCloud)

startingPoint = np.array([0., 0., 0.])

