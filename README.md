# VoroLite: 3D Ray-Tracing with Voronoi Grids

**VoroLite** is a Python-based simulation for 3D ray-tracing in a domain defined by a Voronoi tesselation. Rays are traced through the domain, and the simulation calculates their optical depth as they traverse the Voronoi cells.

## Features
- Simulates ray-tracing in 3D space.
- Uses **Voronoi tesselation** for spatial discretization.
- Tracks optical depth as rays travel through cells with different densities.

## Example Usage

1. **Clone the repository**:

   ```bash
   git clone https://github.com/yourusername/VoroLite.git
   cd VoroLite

2. **Run the Simulations**:

   ```bash
   python VoroLite.py
   
This will simulate ray-tracing starting from the origin (0, 0, 0) in a 3D domain defined by domainMin = -0.5 and domainMax = 0.5. You can adjust the parameters such as the number of rays (nRays) and the number of points in the domain (numPoints) in the script.
