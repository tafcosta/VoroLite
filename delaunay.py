import numpy as np
import matplotlib.pyplot as plt

from scipy.spatial import Delaunay

class Blocks:
    def __init__(self, numBlocks, domainMin, domainMax):

        self.dx = (domainMax - domainMin)/numBlocks
        self.numBlocks = numBlocks
        self.domainMin = domainMin
        self.domainMax = domainMax
        self.totalNumBlocks = numBlocks**3
        
        self.centers = np.zeros((self.totalNumBlocks, 3))
        self.index = np.zeros(self.totalNumBlocks)
        
        iCount = 0
        for iCell in range(numBlocks):
            for jCell in range(numBlocks):
                for kCell in range(numBlocks):
        
                    self.centers[iCell*numBlocks**2 + jCell*numBlocks + kCell] = [domainMin + (0.5 + iCell) * self.dx, domainMin + (0.5 + jCell) * self.dx, domainMin + (0.5 + kCell) * self.dx] 
                    self.index[iCell*numBlocks**2 + jCell*numBlocks + kCell] = iCount
                    iCount += 1
                                
def findHostCell(queryPointPosition, domainBlocks, domainMin, domainMax):

    for i in range(3):
        if(queryPointPosition[i] > domainMax or queryPointPosition[i] < domainMin):
            return -1

    blockXIndex = np.floor((queryPointPosition[0] - domainMin) / domainBlocks.dx)
    blockYIndex = np.floor((queryPointPosition[1] - domainMin) / domainBlocks.dx)
    blockZIndex = np.floor((queryPointPosition[2] - domainMin) / domainBlocks.dx)
        
    return 0




numBlocks =  8
domainMin = -1.0
domainMax =  1.0

# Create a Blocks object
blocks = Blocks(numBlocks, domainMin, domainMax)


num_points = 32
x = np.random.uniform(domainMin, domainMax, num_points)
y = np.random.uniform(domainMin, domainMax, num_points)
z = np.random.uniform(domainMin, domainMax, num_points)

pointCloud    = np.column_stack((x, y, z))
triangulation = Delaunay(pointCloud)

startingPoint = np.array([0., 0., 0.])

