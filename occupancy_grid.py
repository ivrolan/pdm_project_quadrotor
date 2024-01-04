import numpy as np
import matplotlib.pyplot as plt

class OccGrid3D:

    def __init__(self, shape: tuple, origin: tuple, resolution : float):
        """
            Creates an Occupancy Grid of (depthX, widthY, heightZ) dimensions with resolution @resolution starting from coordinates @origin

            Example:
            If we access to a cell at @origin coordinates, this cell would have indices [0,0,0]. Therefore, accessing a cell at coordinates
            @origin + @shape means accesing at cell [-1, -1, -1], that is, the last cell in the occupancy grid
        """


        # TODO: note: for bigger areas with finer resolutions, it is possible that this matrix occupies a lot of memory and is slow to access
        # would a linear array be faster? 

        assert shape[0] > 0.
        assert shape[1] > 0.
        assert shape[2] > 0.

        self.occ_grid  = np.zeros((int(shape[0] / resolution), int(shape[1] / resolution), int(shape[2] / resolution)), dtype=bool)
        self.dimensions = shape
        self.resolution = resolution
        self.origin = origin
    
    def inOccGrid(self, world_coords : tuple) -> bool:
        if world_coords[0] >= self.origin[0] + self.dimensions[0] or world_coords[0] < self.origin[0]:
            return False
        if world_coords[1] >= self.origin[1] + self.dimensions[1] or world_coords[1] < self.origin[1]:
            return False
        if world_coords[2] >= self.origin[2] + self.dimensions[2] or world_coords[2] < self.origin[2]:
            return False
        return True
    
    def getIndicesAt(self, world_coords : tuple):
        """
            Return the indices of the cell that contains the requested @world_coords 
        """

        if self.inOccGrid(world_coords):
            # if it is inside dimensions compute the requested cell indices
            x_index = int((world_coords[0] - self.origin[0]) / self.resolution)
            y_index = int((world_coords[1] - self.origin[1]) / self.resolution)
            z_index = int((world_coords[2] - self.origin[2]) / self.resolution)

            if x_index == self.occ_grid.shape[0]:
                print(f"warning: x_index is {x_index}, as world_coords[0] is {world_coords[0]}, origin_x: {self.origin[0]} and res is {self.resolution}")
            if y_index == self.occ_grid.shape[1]:
                print(f"warning: y_index is {y_index}, as world_coords[1] is {world_coords[1]}, origin_y: {self.origin[1]} and res is {self.resolution}")
            if z_index == self.occ_grid.shape[2]:
                print(f"warning: z_index is {z_index}, as world_coords[2] is {world_coords[2]}, origin_z: {self.origin[2]} and res is {self.resolution}")

            return (x_index, y_index, z_index)
        else:
            return None

    def occupyCoords(self, world_coords : tuple):

        indices = self.getIndicesAt(world_coords)
        
        if indices == None:
            raise IndexError(f"The requested world coordinates {world_coords} do not refer to any cell in the occupancy grid with origin {self.origin} and dimensions {self.dimensions}")
        else:
            self.occ_grid[indices[0], indices[1], indices[2]] = True
    
    def freeCoords(self, world_coords : tuple):

        indices = self.getIndicesAt(world_coords)
        
        if indices == None:
            raise IndexError(f"The requested world coordinates {world_coords} do not refer to any cell in the occupancy grid with origin {self.origin} and dimensions {self.dimensions}")
        else:
            self.occ_grid[indices[0], indices[1], indices[2]] = False

    def getOccupancyCell(self, indices : tuple) -> bool:
        # note: this is different from self.isCellOccupied() as the values in the OccupancyGrid can be between 0 and 255
        return self.occ_grid[indices[0], indices[1], indices[2]]
    
    def isCellOccupied(self, indices : tuple) -> bool:
        # note: if the values in occ_grid go from 0 to 255 we can specify a threshold to consider it occupied
        return self.getOccupancyCell(indices)
    
    def isCoordOccupied(self, world_coords: tuple):
        """
            Returns if the position XYZ specified by @world_coords is occupied or not
        """
        indices = self.getIndicesAt(world_coords)
        if indices == None:
            raise IndexError(f"The requested world coordinates {world_coords} do not refer to any cell in the occupancy grid with origin {self.origin} and dimensions {self.dimensions}")
        else:
            return self.isCellOccupied(indices)
    
    def plot(self):
        """
            Simple visualization of the cells in Matplotlib
        """
        ax = plt.figure().add_subplot(projection='3d')
        ax.voxels(self.occ_grid, edgecolor='k')
        ax.set_aspect('equal', adjustable='box')
        plt.show()