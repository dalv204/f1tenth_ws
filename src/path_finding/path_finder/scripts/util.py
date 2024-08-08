"""
File for holding custom data structures and common functions 

"""

class Occupancy:
    """ 
    Class for managing the occupancy layers 
    """

    def __init__(self, scale, dimensions, origin, static_grid):
        """
        Args: 
            dimensions - tuple width and height
            static_grid - tuple of all map locations
        """
        
        # with open("data_file.json", 'w') as dt:
        #     dt.write(str(list(static_grid)))
        #     print("wrote the file")
        # dt.close()
        
        self.dynamic_grid = set()
        self.scale = scale
        self.width = dimensions[0]
        self.height = dimensions[1]
        self.origin = origin
        self.static_grid = self.initialize_static(static_grid)
        print(self.scale, self.width,self.height, self.origin)

    # possible functions I will need

    def __add__(self, pos):
        """ dunder add for dynamic graph """
        if pos[0]*pos[1] < self.width*self.height:
            self.dynamic_grid.add(pos)
        else:
            raise IndexError("Out of the map's range") 
    
    def __contains__(self, pos):
        """ checks if a position is open or not """
        return pos in self.static_grid | self.dynamic_grid

    
    # def __del__(self, pos):
    #     """ if something was present in the dynamic before,
    #     but is no longer, remove it 
    #     """
    #     if pos in self.dynamic_grid:
    #         self.dynamic_grid.remove(pos)


    def pos_to_coord(self, x, y):
        """ finds pixel x and y given real world position"""
        min_x = self.origin[0]
        pos_x = int(max(min((x - min_x)//self.scale, self.width), 0))
        min_y = self.origin[1]
        pos_y = int(max(min((y-min_y)//self.scale, self.height), 0))
        # ensure it stays within bounds ^^ 
        return pos_x, pos_y

    def initialize_static(self, static_grid):
        """  data is in row-major order
         returns pixel coordinates self.width x self.height 
        - unknown points will be left as unoccupied"""

        return set((index - (self.width * (y:=index//self.width)), y) for 
                   index, value in enumerate(static_grid) if value == 100)
    
        # modulo operator is slow, mitigating by seeing where the y value 
        # is... 
        
    def random_point(self, limiter):
        """ 
        finds a random point in the 'open space' 
        """

        value = np.random.uniform(0, limiter)
        if value in self:
            # not free - search again 
            value = self.random_point(limiter)
        return int(value) 

    
    def coord_to_pos(self, node):
        """ transforms into the map's perspective 
        TODO - this may be better suited outside the class
        """
        min_x = self.origin[0]
        min_y = self.origin[1]
        if hasattr(node, 'x'):
            x = (node.x * self.scale)+min_x
            y = (node.y* self.scale)+min_y
        else:
            # assume can separate by index
            x = (node[0] * self.scale)+min_x
            y = (node[1]* self.scale)+min_y

        return (x,y)




