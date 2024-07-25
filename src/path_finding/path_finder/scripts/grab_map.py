# file to help me practice grabbing information from map :)
import numpy as np

# static_struct = np.array([[]])

# def static(...):
#     """ 
#     function grabs information from map to create 'static' layer
#     want a quick lookup idea
#     """

# def dynamic(...):
#     """ 
#     could be created by processing the LiDar Data and 
#     adding something that was not present in the static environment 
#     (if the object is far enough away from other points so that it 
#     cannot be the same point read incorrectly)
#     """

# TODO - write a code that computes the path of travel ÖÖÖ 

def find_grid_path(start, stop, width):
    """ finds the affected coordinates """
    direct_vector = stop-start
    length = int(np.linalg.norm(direct_vector))
    print(f"{length=}")
    direct_vector = direct_vector / length
    offset = int(width / 2)
    print(f"{offset=}")
    path = []
    square = [np.array([x,y]) for x in range(start[0]-offset, start[0]+(offset+1)) for y in range(start[1]-offset, start[1]+(offset+1))]
    path = set(tuple((array+(mult*direct_vector)).astype(int)) for mult in range(1, length+1) for array in square)
    # might be easiest to populate a square and run the for loop on each point in the square
    print(path)
    # for 


start = np.array([31, 27])
stop = np.array([30, 27])
width = .3/.05

find_grid_path(start, stop, width)


def random_point(limiter):
        """ 
        finds a random point in the 'open space' 
        """
        value = np.random.uniform(0, limiter)
        return int(value) 

def sample():
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        x = random_point(142) 
        y = random_point(125)

        return (x, y)

print(sample())


import numpy as np

length = np.linalg.norm([-2-3, -5-18])
print(f"LENGTH IS {length}")