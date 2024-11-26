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


domainMin = -1.0
domainMax =  1.0
startingPoint  = np.array([0., 0.])
numPoints = 32

domain = Domain(domainMin, domainMax, 2)

x = np.random.uniform(domainMin, domainMax, numPoints)
y = np.random.uniform(domainMin, domainMax, numPoints)
z = np.random.uniform(domainMin, domainMax, numPoints)
pointCloud    = np.column_stack((x, y))

nRays = 1
rays = Rays(nRays, startingPoint)

triangulation = Delaunay(pointCloud)
voronoi = Voronoi(pointCloud)

indptr,indices = triangulation.vertex_neighbor_vertices
numberNeighbours = indptr[1:]-indptr[:-1]

startingCell   = findHostCell(startingPoint, domain, voronoi)

propagate = True
while(propagate):

    s = 1.e10
    for iNeighbour in range(numberNeighbours[startingCell]):
        normalVector = pointCloud[startingCell] - pointCloud[indices[indptr[startingCell] + iNeighbour]]
        pointOnInterface = (pointCloud[startingCell] + pointCloud[indices[indptr[startingCell] + iNeighbour]]) / 2.0

        sTmp = np.dot(normalVector,  (pointOnInterface - startingPoint)) / np.dot(normalVector, np.array([rays.kx, rays.ky]))
        if sTmp > 0:
            if sTmp < s:
                s = sTmp
        
        print(sTmp, s)

    propagate = False
