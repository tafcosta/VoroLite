import numpy as np
import matplotlib.pyplot as plt
import sys

from scipy.spatial import Delaunay, Voronoi, voronoi_plot_2d

class Rays:
    def __init__(self, nRays, startingPosition):
        
        self.nRays = nRays
        self.kx = np.zeros(nRays)
        self.ky	= np.zeros(nRays)
        self.xPos = np.zeros(nRays)
        self.yPos = np.zeros(nRays)
        self.opticalDepth = np.zeros(nRays)
        self.nTraversedCells = np.zeros(nRays)
        self.traversedCells = np.empty(nRays, dtype=object)
        
        for iRay in range(nRays):
            self.phi = np.random.uniform(0, 2 * np.pi)
            self.kx[iRay]   = np.cos(self.phi)
            self.ky[iRay]   = np.sin(self.phi)
            self.xPos[iRay] = startingPosition[0]
            self.yPos[iRay] = startingPosition[1]

class Domain:
    def __init__(self, domainMin, domainMax, nDim):
        self.domainMin = domainMin
        self.domainMax = domainMax
        self.nDim = nDim
            
def findHostCell(queryPointPosition, domain, vor):

    for i in range(domain.nDim):
        if(queryPointPosition[i] > domain.domainMax or queryPointPosition[i] < domain.domainMin):
            return -1
        
    return np.argmin(np.linalg.norm(vor.points - queryPointPosition, axis=1))

def getBoundaryCells(voronoi):
    isBoundary = np.zeros(len(voronoi.point_region))
    for iCell in np.arange(len(voronoi.point_region)):
        isBoundary[iCell] = int(-1 in voronoi.regions[voronoi.point_region[iCell]])

    return isBoundary
        
domainMin = -0.5
domainMax =  0.5
startingPoint  = np.array([0., 0.])
numPoints = 256

domain = Domain(domainMin, domainMax, 2)

x = np.random.uniform(domain.domainMin, domain.domainMax, numPoints)
y = np.random.uniform(domain.domainMin, domain.domainMax, numPoints)

pointCloud    = np.column_stack((x, y))
density       = np.where(np.sqrt(x**2 + y**2) <= 0.1, 1, 0.0)




nRays = 10000
rays  = Rays(nRays, startingPoint)

triangulation    = Delaunay(pointCloud)
voronoi          = Voronoi(pointCloud)

indptr,indices   = triangulation.vertex_neighbor_vertices
numberNeighbours = indptr[1:]-indptr[:-1]
boundaryFlag     = getBoundaryCells(voronoi)
startingCell     = findHostCell(startingPoint, domain, voronoi)

for iRay in range(rays.nRays):
    propagate = True
    iCell = startingCell
        
    while(propagate):
        distanceToExit = sys.float_info.max
        exitCell  = 0

        numberPossibleNeighbours =  0
        for iNeighbour in range(numberNeighbours[iCell]):
            normalVector       = pointCloud[iCell] - pointCloud[indices[indptr[iCell]:indptr[iCell + 1]][iNeighbour]]
            pointOnInterface   = (pointCloud[iCell] + pointCloud[indices[indptr[iCell]:indptr[iCell + 1]][iNeighbour]]) / 2.0
            denominator        = np.dot(normalVector, np.array([rays.kx[iRay], rays.ky[iRay]]).ravel())

            if(np.abs(denominator) > sys.float_info.epsilon):
                distanceToExitTmp  = np.dot(normalVector,  (pointOnInterface - np.array([rays.xPos[iRay], rays.yPos[iRay]]).ravel())) / denominator
            else:
                continue

            if ((distanceToExitTmp < distanceToExit) and (distanceToExitTmp > 0)):
                distanceToExit = distanceToExitTmp
                exitCell       = indices[indptr[iCell]:indptr[iCell + 1]][iNeighbour]

                numberPossibleNeighbours += 1

            #print(distanceToExitTmp, numberNeighbours[iCell], iCell)

                
        if(numberPossibleNeighbours == 0):
            raise ValueError(f"Error: Loop terminated because no elligible neighbours were found.")

        rays.nTraversedCells[iRay] += 1
        
        rays.xPos[iRay] += rays.kx[iRay] * distanceToExit * 1.0000001
        rays.yPos[iRay] += rays.ky[iRay] * distanceToExit * 1.0000001
        rays.opticalDepth[iRay] += distanceToExit * density[iCell]

        rays.traversedCells[iRay] = np.append(rays.traversedCells[iRay], iCell)

        iCell = exitCell
        
        if(boundaryFlag[iCell] != 0):
            propagate = False
            rays.traversedCells[iRay] = np.append(rays.traversedCells[iRay], exitCell)

            distanceToExit = sys.float_info.max

            if(rays.kx[iRay] > sys.float_info.epsilon):
                distanceToExitTmp = (domain.domainMin - rays.xPos[iRay]) / rays.kx[iRay]
                if ((distanceToExitTmp < distanceToExit) and (distanceToExitTmp > sys.float_info.epsilon)):
                    distanceToExit = distanceToExitTmp
                    
                distanceToExitTmp = (domain.domainMax - rays.xPos[iRay]) / rays.kx[iRay]
                if ((distanceToExitTmp < distanceToExit) and (distanceToExitTmp > sys.float_info.epsilon)):
                    distanceToExit = distanceToExitTmp

            if(rays.ky[iRay] > sys.float_info.epsilon):
                distanceToExitTmp = (domain.domainMin - rays.yPos[iRay]) / rays.ky[iRay]
                if ((distanceToExitTmp < distanceToExit) and (distanceToExitTmp > sys.float_info.epsilon)):
                    distanceToExit = distanceToExitTmp

                distanceToExitTmp = (domain.domainMax - rays.yPos[iRay]) / rays.ky[iRay]
                if ((distanceToExitTmp < distanceToExit) and (distanceToExitTmp > sys.float_info.epsilon)):
                        distanceToExit = distanceToExitTmp
