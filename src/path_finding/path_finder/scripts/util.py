"""
File for holding custom data structures and common functions 

"""
import numpy as np

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
        self.car_width = 0.4 # 40 centimeter
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
    
    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        width = self.car_width / self.scale  # gives us the car width in terms of coordinate squares 
        # assuming new_node is the target (stop) and near_pos is start
        direct_vector = new_node - nearest_node
        length = int(np.linalg.norm(direct_vector))
        direct_vector = (direct_vector / length) if length>0 else np.array([0,0])
        offset = int(width/2)
        square = [np.array([x,y]) for x in range(nearest_node.x-offset, nearest_node.x+(offset+1)) 
                  for y in range(nearest_node.y-offset, nearest_node.y+(offset+1))]
        # TODO - PROBABLY A QUICKER WAY TO DO THE SQUARE
        path = set(tuple((array+(mult*direct_vector)).astype(int)) for mult in range(1, length+1) for array in square)
        # TODO - could potentially be quicker to check while creating the path, depending on how large the path is
        return any(location in self for location in path)


class TreeNode:
    # TODO - ADD FUNCTIONS TO EASILY SUBTRACT TREE NODES 
    # TODO - ALSO SEE IF YOU CAN ADD SOME SORT OF LEN() 
    # TO FIND THE DISTANCE BETWEEN THEM
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = 0 if parent is None else float('inf') # only used in RRT*
        # self.is_root = False
    
    def __sub__(self, other):
        """ subtracts the difference between a node and other item"""
        # need to make sure that other has len 2
        if hasattr(other, 'x'):
            # get the attributes
            return np.array([self.x - other.x, self.y-other.y])

        else:
            # probably a numpy array
            return np.array([self.x - other[0], self.y - other[1]])
        
    def __mul__(self, other):
        """ runs multiplication """

        if hasattr(other, 'x'):
            # maybe works somewhat like dot product?
            return np.array([self.x * other.x, self.y*other.y])
        else:
            # integer or float multiplication
            return np.array([self.x * other, self.y * other])
        
    def __add__(self, other):
        """ adds two together, +1 func call compare to sub"""
        return self - (-1*other)
    
    def __div__(self, other):
        raise NotImplementedError
    
    def get_coord(self):
        """ returns the current coords in tuple form """
        return np.array([self.x, self.y])
    
    def __repr__(self):
        return f"({self.x}, {self.y})"

