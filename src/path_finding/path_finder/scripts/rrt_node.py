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
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import csv  # may need this?
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid

# TODO: import as you need

# class def for tree nodes
# It's up to you if you want to use this
class TreeNode():
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = 0 if parent is None else float('inf') # only used in RRT*
        # self.is_root = False

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

        # TODO: create subscribers
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
        self.occupancy_grid = None
        self.tree = []
        self.goal = (10,10) # goal for example (CHANGE FOR ACTUAL?)
        self.goal_tolerance = 0.5

        # TODO: create a drive message publisher, and other publishers that you might need

        # class attributes
        # TODO: maybe create your occupancy grid here

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """
        pass

    def pose_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        # pretty sure pose_msg.pose.pose.position.x is required (remove verify later?)
        if not self.tree:
            start = TreeNode(pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y)
            self.tree.append(start)

        for _ in range(100): # number of iterations
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
                    if cost < min_cost and self.check_collision(neighbor, new_node):
                        min_cost = cost
                        min_cost_node = neighbor
                new_node.parent = min_cost_node
                new_node.cost = min_cost
                self.tree.append(new_node)

                for neighbor in neighbors:
                    cost = new_node.cost + self.line_cost(new_node, neighbor)
                    if cost < neighbor.cost and self.check_collision(new_node, neighbor):
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
        x = np.random.uniform(-10,10)
        y = np.random.uniform(-10,10)

        # TODO - this is a simplified representation for now, but need to make
        # TODO - a more complex algorithm to determine what constitutes free space
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
        return True
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
        return False

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