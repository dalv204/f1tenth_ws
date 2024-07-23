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
    direct_vector = direct_vector / length
    offset = width // 2
    path = []
    square = [np.array([x,y]) for x in range(start[0]-offset, start[0]+(offset+1)) for y in range(start[1]-offset, start[1]+(offset+1))]
    path = set(tuple((array+(mult*direct_vector)).astype(int)) for mult in range(1, length+1) for array in square)
    # might be easiest to populate a square and run the for loop on each point in the square
    print(path)
    # for 

start = np.array([0,0])
stop = np.array([5,7])
width = 4

find_grid_path(start, stop, width)

