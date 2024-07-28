# # file to help me practice grabbing information from map :)
# import numpy as np

# # static_struct = np.array([[]])

# # def static(...):
# #     """ 
# #     function grabs information from map to create 'static' layer
# #     want a quick lookup idea
# #     """

# # def dynamic(...):
# #     """ 
# #     could be created by processing the LiDar Data and 
# #     adding something that was not present in the static environment 
# #     (if the object is far enough away from other points so that it 
# #     cannot be the same point read incorrectly)
# #     """

# # TODO - write a code that computes the path of travel ÖÖÖ 

# def find_grid_path(start, stop, width):
#     """ finds the affected coordinates """
#     direct_vector = stop-start
#     length = int(np.linalg.norm(direct_vector))
#     print(f"{length=}")
#     direct_vector = direct_vector / length
#     offset = int(width / 2)
#     print(f"{offset=}")
#     path = []
#     square = [np.array([x,y]) for x in range(start[0]-offset, start[0]+(offset+1)) for y in range(start[1]-offset, start[1]+(offset+1))]
#     path = set(tuple((array+(mult*direct_vector)).astype(int)) for mult in range(1, length+1) for array in square)
#     # might be easiest to populate a square and run the for loop on each point in the square
#     print(path)
#     # for 


# start = np.array([31, 27])
# stop = np.array([30, 27])
# width = .3/.05

# find_grid_path(start, stop, width)


# def random_point(limiter):
#         """ 
#         finds a random point in the 'open space' 
#         """
#         value = np.random.uniform(0, limiter)
#         return int(value) 

# def sample():
#         """
#         This method should randomly sample the free space, and returns a viable point

#         Args:
#         Returns:
#             (x, y) (float float): a tuple representing the sampled point

#         """
#         x = random_point(142) 
#         y = random_point(125)

#         return (x, y)

# print(sample())


# import numpy as np

# length = np.linalg.norm([-2-3, -5-18])
# print(f"LENGTH IS {length}")

import numpy as np


def identify_critical_points(self, ang_tolerance, dist_tolerance):
    """ 
    simplifies the path found from RRT to what is needed 
    returns the critical points :)
    """
    critical_points = [coords[0]]
    length = len(coords)
    for i in range(1, length-1):
        v1 = np.array(coords[i]) - np.array(coords[i-1])
        v2 = np.array(coords[i+1]) - np.array(coords[i])
        
        angle = calculate_angle(v1, v2)
        if angle > np.radians(ang_tolerance):
            v21 = v2 - v1
            perp = np.linalg.norm(v21)/v21
            bar = set()
            new_pos = TreeNode(coords[i].x, coords[i].y)
            while not check_collision(coords[i], new_pos):
                mult +=1
                bar.add((new_pos.x, new_pos.y))
                new_pos.x = int(new_pos.x *perp[0]*mult)
                new_pos.y = int(new_pos.y *perp[1]*mult)

            while not check_collision(coords[i], new_pos):
                mult -=1
                bar.add((new_pos.x, new_pos.y))
                new_pos.x = int(new_pos.x *perp[0]*mult)
                new_pos.y = int(new_pos.y *perp[1]*mult)
            critical_points.append((coords[i], bar))
        else:
            dist = np.linalg.norm(np.array[coords[i]] - np.array(critical_points[-1]))
            if dist > dist_tolerance:
                  critical_points.append(coords[i])
        critical_points.append(coords[-1])

def calculate_angle(v1, v2):
        """ calculates the angle between consecutive points """
        unit_v1 = v1 / np.linalg.norm(v1)
        unit_v2 = v2 / np.linalg.norm(v2)
        dot_product = np.dot(unit_v1, unit_v2)
        return np.arccos(dot_product)


def create_bars():
      """ given points, creates the bars """


coords = [
        (0.0, 0.0),
        (1.0, 1.0),
        (2.0, 2.0),
        (3.0, 1.0),
        (4.0, 0.0),
        (5.0, -1.0),
        (6.0, -2.0),
        (7.0, -1.0),
        (8.0, 0.0),
        (9.0, 1.0),
        (10.0, 2.0)
    ]