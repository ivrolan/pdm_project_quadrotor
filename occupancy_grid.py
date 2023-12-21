import numpy as np

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
        self.occ_grid  = np.zeros((shape[0] // resolution, shape[1] // resolution, shape[2] // resolution), dtype=bool)
        self.dimensions = shape
        self.resolution = resolution
        self.origin = origin
    
    def inOccGrid(self, world_coords : tuple) -> bool:
        if world_coords[0] > self.origin[0] + self.dimensions[0] or world_coords[0] < self.origin[0]:
            return False
        if world_coords[1] > self.origin[1] + self.dimensions[1] or world_coords[1] < self.origin[1]:
            return False
        if world_coords[2] > self.origin[2] + self.dimensions[2] or world_coords[2] < self.origin[2]:
            return False
        return True
    
    def getIndicesAt(self, world_coords : tuple):
        """
            Return the indices of the cell that contains the requested @world_coords 
        """

        if self.inOccGrid(world_coords):
            # if it is inside dimensions compute the requested cell indices
            x_index = (world_coords[0] - self.origin[0]) // self.resolution
            y_index = (world_coords[1] - self.origin[1]) // self.resolution
            z_index = (world_coords[2] - self.origin[2]) // self.resolution
        
            return (x_index, y_index, z_index)
        else:
            return None
        
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
            raise IndexError("The requested world coordinates do not refer to any cell in the occupancy grid")
        else:
            return self.isCellOccupied(indices)
    
    """
        TODO: are they necessary or should we use directly self.occ_grid

        def occupyCell(self, indices : tuple) -> bool:
            pass
        def occupyCoords(self, world_coords: tuple):
            pass
    """
    