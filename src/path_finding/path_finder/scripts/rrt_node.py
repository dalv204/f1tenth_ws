"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import csv  # may need this?
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid

# TODO: import as you need

# class def for tree nodes
# It's up to you if you want to use this
class TreeNode:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = 0 if parent is None else float('inf') # only used in RRT*
        # self.is_root = False

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

        self.static_grid = self.initialize_static(static_grid)
        self.dynamic_grid = set()
        self.scale = scale
        self.width = dimensions[0]
        self.height = dimensions[1]
        self.origin = origin

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

    
    def __del__(self, pos):
        """ if something was present in the dynamic before,
        but is no longer, remove it 
        """
        if pos in self.dynamic_grid:
            self.dynamic_grid.remove(pos)

    def pos_to_coord(self, x, y):
        """ finds pixel x and y given real world position"""
        min_x = self.origin[0]
        pos_x = max(min((x - min_x)//self.scale, self.width), 0)
        min_y = self.origin[1]
        pos_y = max(min((y-min_y)//self.scale, self.height), 0)
        # ensure it stays within bounds ^^ 
        return pos_x, pos_y

    def initialize_static(self, static_grid):
        """  data is in row-major order
         returns pixel coordinates self.width x self.height 
        - unknown points will be left as unoccupied"""

        return set((index - (self.width * (y:=index//self.width)), y) for 
                   index, value in enumerate(static_grid) if value == 100)
    
            # modulo operator is slow, could mitigate this by seeing where the y value 
            # is... 
        
    def random_point(self, limiter):
        """ 
        finds a random point in the 'open space' 
        """
        value = np.random.uniform(0, limiter)
        if value in self:
            # not free - search again 
            value = self.random_point(limiter)
        return value 
    
    def transform_pos(self):
        """ transforms into the car's perspective 
        TODO - this may be better suited outside the class
        """

    
    def un_transform(self):
        """ transforms into the map's perspective 
        TODO - this may be better suited outside the class
        """

# class def for RRT
class RRT(Node):
    """
    have to modify this code skeleton to fit the RRT* algorithm
    current skeleton is better suited for the RRT algorithm

    """
    def __init__(self):
        super().__init__('rrt_node')
        pose_topic = "ego_racecar/odom"
        scan_topic = "/scan"

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        self.map_sub_ = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10,
        )

        self.pose_sub_ = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_callback,
            1)

        self.scan_sub_ = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            1)

        # publishers
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.Grid = None  
        self.mapped=False
        self.tree = []
        self.goal = (10,10) # goal for example (CHANGE FOR ACTUAL?)
        self.goal_tolerance = 0.5
        self.car_width = .3 # meters

        # TODO: create a drive message publisher, and other publishers that you might need

        # class attributes
        # TODO: maybe create your occupancy grid here
    
    def map_sub_(self, msg):
        """ only want to create one instance"""
        if not self.mapped: # only want to create 1 class instance
            # I believe width corresponds to x and height to y
            # TODO - assumes "static" map doesn't update
            self.Grid = \
                Occupancy(msg.info.resolution, 
                          (msg.info.width, msg.info.height), msg.info.origin, msg.data )
            self.mapped=True

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:
            2nd layer to the ocupancy grid

        """
        pass

    def get_iteration_count(self):
        """
        calculates reasonable number of iterations for system 
        based on speed and computational resources
        """
        # TODO - incorporate this later
        return 100

    def pose_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        # pretty sure pose_msg.pose.pose.position.x is required (remove verify later?)
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        pos_x, pos_y = self.Grid.pos_to_coord(x,y)

        if not self.tree:
            start = TreeNode(pos_x, pos_y)
            self.tree.append(start)
        else:
            if self.no_longer_valid(pos_x, pos_y):
                self.replan(pos_x, pos_y)
            else:
                self.grow_tree()

    def no_longer_valid(self, x, y):
        """ checks if current tree no longer valid, possible reasons:
        - not where I thought I would be 
        - unexpected obstacle found 
        - perhaps a timing based restart mechanism?
        """
        # TODO - this requires having an estimate of where we should be...
        # could calculate based on speed and the waypoints
        # need more development in other locations before this works
        return False


    def replan(self, x, y):
        """ 
        starts the tree over 
        because of unforseen condition
        """
        del self.tree[:]
        start = TreeNode(x, y)
        self.tree.append(start)
        self.grow_tree()
    
    def grow_tree(self):
        """ expands the current tree"""

        for _ in range(self.get_iteration_count()): # number of iterations
            sampled_point = self.sample()
            nearest_index = self.nearest(self.tree, sampled_point)
            nearest_node = self.tree[nearest_index]
            new_node = self.steer(nearest_node, sampled_point)

            if self.check_collision(nearest_node, new_node):
                neighbors = self.near(self.tree, new_node)
                min_cost_node = nearest_node
                min_cost = nearest_node.cost + self.line_cost(nearest_node, new_node)

                for neighbor in neighbors:
                    cost = neighbor.cost + self.line_cost(neighbor, new_node)
                    if cost < min_cost and not self.check_collision(neighbor, new_node):
                        min_cost = cost
                        min_cost_node = neighbor
                new_node.parent = min_cost_node
                new_node.cost = min_cost
                self.tree.append(new_node)

                for neighbor in neighbors:
                    cost = new_node.cost + self.line_cost(new_node, neighbor)
                    if cost < neighbor.cost and not self.check_collision(new_node, neighbor):
                        neighbor.parent = new_node
                        neighbor.cost = cost
                if self.is_goal(new_node, *self.goal):
                    path = self.find_path(self.tree, new_node)
                    self.publish_path(path)
                    break

    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        x = self.Grid.random_point(self.Grid.width) 
        y = self.Grid.random_point(self.Grid.height)

        return (x, y)

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_index = 0
        min_dist = float('inf')
        for i,node in enumerate(tree):
            dist = LA.norm([sampled_point[0] - node.x, sampled_point[1] - node.y])
            if dist < min_dist:
                min_dist = dist
                nearest_index = i
        return nearest_index

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        # TODO - needs to be mofidified to fit the coordinate stuff
        direction = np.array([sampled_point[0] - nearest_node.x, sampled_point[1]-nearest_node.y])
        length = LA.norm(direction)
        direction = direction/length
        step_size = 0.5
        new_point = (nearest_node.x + direction[0] * step_size, nearest_node.y + direction[1] * step_size)
        return TreeNode(new_point[0], new_point[1], parent=nearest_node)

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
        # take the line into account, but also the size of the car
        # will depend on the resolution of the map
        new_pos = np.array([new_node.x, new_node.y])
        near_pos = np.array([nearest_node.x, nearest_node.y])
        collision_path = ...

        return any(value in self.Grid for value in collision_path)
        # will implement actual collision checking later

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        return LA.norm([goal_x - latest_added_node.x, goal_y - latest_added_node.y]) <= self.goal_tolerance
        # return False

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        node = latest_added_node
        while node is not None:
            path.append(node)
            node = node.parent
        path.reverse() # last added should be the first step :)
        return path


    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return node.cost

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straight line
        Returns:
            cost (float): the cost value of the line
        """
        return LA.norm([n1.x - n2.x, n1.y-n2.y])  
        # for now cost is just determined by distance

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        # make radius 1
        radius = 1.0
        neighbors = []
        for n in tree:
            if LA.norm([node.x - n.x, node.y-n.y]) <= radius:
                # means that the node is "near"
                neighbors.append(n)
        return neighbors
    

    def publish_path(self, path):
        drive_msg = AckermannDriveStamped()
        # TODO - simplified, needs to be calculated
        drive_msg.drive.steering_angle = 0.0
        # TODO - simplified, needs to be calculated
        drive_msg.drive.speed = 1.0 
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("RRT Initialized")
    rrt_node = RRT()
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()