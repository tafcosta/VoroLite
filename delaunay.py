import numpy as np
import matplotlib.pyplot as plt

from scipy.spatial import Delaunay, Voronoi, voronoi_plot_2d

class Rays:
    def __init__(self, nRays, startingPosition):
        
        self.nRays = nRays
        self.kx = np.zeros(nRays)
        self.ky	= np.zeros(nRays)
        self.xPos = np.zeros(nRays)
        self.yPos = np.zeros(nRays)
        self.opticalDepth = np.zeros(nRays)
        
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


domainMin = -0.5
domainMax =  0.5
startingPoint  = np.array([0., 0.])
numPoints = 512

domain = Domain(domainMin, domainMax, 2)

x = np.random.uniform(domain.domainMin, domain.domainMax, numPoints)
y = np.random.uniform(domain.domainMin, domain.domainMax, numPoints)
z = np.random.uniform(domain.domainMin, domain.domainMax, numPoints)
pointCloud    = np.column_stack((x, y))

density = np.where(np.abs(y) <= 0.3, 1, 1)

nRays = 10
rays = Rays(nRays, startingPoint)

triangulation = Delaunay(pointCloud)
voronoi = Voronoi(pointCloud)

indptr,indices = triangulation.vertex_neighbor_vertices
numberNeighbours = indptr[1:]-indptr[:-1]

startingCell   = findHostCell(startingPoint, domain, voronoi)



for iRay in range(rays.nRays):
    propagate = True

    iCell = startingCell
    while(propagate):
        distanceToExit = 1.e10
        exitCell  = 0

        for iNeighbour in range(numberNeighbours[iCell]):
            normalVector = pointCloud[iCell] - pointCloud[indices[indptr[iCell]:indptr[iCell + 1]][iNeighbour]]
            pointOnInterface = (pointCloud[iCell] + pointCloud[indices[indptr[iCell]:indptr[iCell + 1]][iNeighbour]]) / 2.0
            distanceToExitTmp  = np.dot(normalVector,  (pointOnInterface - np.array([rays.xPos[iRay], rays.yPos[iRay]]).ravel())) / np.dot(normalVector, np.array([rays.kx[iRay], rays.ky[iRay]]).ravel())

            if (distanceToExitTmp > 1.e-7) and (distanceToExitTmp < distanceToExit):
                distanceToExit = distanceToExitTmp
                exitCell       = indices[indptr[iCell]:indptr[iCell + 1]][iNeighbour]

        if distanceToExit > np.sqrt(2 * ((domain.domainMax - domain.domainMin)/2)**2):
            distanceToExitTmp = (domain.domainMin - rays.xPos[iRay]) / rays.kx[iRay]
            if (distanceToExitTmp > 1.e-7) and (distanceToExitTmp < distanceToExit):
                distanceToExit = distanceToExitTmp
            
            distanceToExitTmp =	(domain.domainMax - rays.xPos[iRay]) / rays.kx[iRay]
            if (distanceToExitTmp > 1.e-7) and (distanceToExitTmp < distanceToExit):
                distanceToExit = distanceToExitTmp

            distanceToExitTmp =	(domain.domainMin - rays.yPos[iRay]) / rays.ky[iRay]
            if (distanceToExitTmp > 1.e-7) and (distanceToExitTmp < distanceToExit):
                distanceToExit = distanceToExitTmp
            
            distanceToExitTmp = (domain.domainMax - rays.yPos[iRay]) / rays.ky[iRay]
            if (distanceToExitTmp > 1.e-7) and (distanceToExitTmp < distanceToExit):
                distanceToExit = distanceToExitTmp

        rays.xPos[iRay] += rays.kx[iRay] * distanceToExit
        rays.yPos[iRay] += rays.ky[iRay] * distanceToExit
        rays.opticalDepth[iRay] += distanceToExit * density[iCell]
        
        if rays.xPos[iRay] < domain.domainMin or rays.xPos[iRay] > domain.domainMax:
            propagate = False
        if rays.yPos[iRay] < domain.domainMin or rays.yPos[iRay] > domain.domainMax:
            propagate = False

        iCell = exitCell
