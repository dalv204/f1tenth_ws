#!/usr/bin/env python3

import numpy as np
from numpy import linalg as LA
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PointStamped, Pose, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker
import csv  # may need this?
import ast
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import time

# TODO: import as you need

# class def for tree nodes
# It's up to you if you want to use this


# TODO - so, I think with the global node, I should be able to pick a point, and draw a line between 
# TODO - that "checkpoint" and the borders of the track; 

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
        x = (node.x * self.scale)+min_x
        y = (node.y* self.scale)+min_y
        return (x,y)

class DualSearch(Node):
    """ class to handle global and local search """
    def __init__(self):
        super().__init__('dual_search_node')
        pose_topic = "/ego_racecar/odom"
        scan_topic = "/scan"
        clicked_topic = "/clicked_point"
        
        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # self.map_sub_ = self.create_subscription(
        #     OccupancyGrid,
        #     '/map',
        #     self.map_callback,
        #     10,
        # )

        # TODO- PAUSE MAP SUB FOR TESTING

        self.pose_sub_ = self.create_subscription(
            Odometry,
            pose_topic,
            self.pose_callback,
            1)

        self.scan_sub_ = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            1)
        
        self.point_sub = self.create_subscription(
            PointStamped,
            clicked_topic,
            self.goal_callback,
            10
        )

        # publishers
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.waypoint_pub = self.create_publisher(
            String,
            "/custom_waypoints",
            10
        )

        self.marker_pub = self.create_publisher(Marker, 'waypoints_marker', 10)

        # TODO - include the other listeners I have here

        self.global_path=(False)
        self.local_path = (True)
        self.last_global_update = time.time()
        self.global_update_interval = 5.0

        # TODO - want to create the grids here so they 
        # TODO -- can both access the occupancy grid

        self.global_planner = RRT()
        self.local_planner = RRT()
        self.Grid = Occupancy()
        self.current_position = None, None
        self.goal = None
        self.goal_tolerance=None
        self.yaw = None
        self.coord_x,self.coord_y = None, None
        # self.goal represents the "global" goal

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:
            2nd layer to the ocupancy grid

        """
        # logic will be implemented later perhaps... is a little complicated 
        # because it depends on the accuracy of positioning system... 
        # might be better for shorter term obstacles , like cars
        # like dynamic should be used for moving objects / quick reactions.. focus on latr
        # assert NotImplementedError

        pass

    def map_callback(self, msg):
        """ only want to create one instance"""
        if not self.mapped: # only want to create 1 class instance
            # I believe width corresponds to x and height to y
            # TODO - assumes "static" map doesn't update
            self.Grid = Occupancy(msg.info.resolution, 
                            (msg.info.width, msg.info.height), 
                            (msg.info.origin.position.x,
                            msg.info.origin.position.y),
                            msg.data )
            print(self.Grid)
            self.goal_tolerance = int(0.5/self.Grid.scale)
            self.mapped=True
    
    def pose_callback(self, pose_msg):
        self.yaw = self.quaternion_to_yaw(pose_msg.pose.pose.orientation)
        self.update_current_position(pose_msg)
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        self.coord_x, self.coord_y = self.Grid.pose_to_coord(x,y)
        # updates pos for both trees ^^

        current_time = time.time()
        if (current_time - self.last_global_update > self.global_update_interval) \
                or self.global_plan is None:
            self.last_global_update = current_time
            self.update_global_path()

        self.update_local_path()

        self.publish_local_path()
    
    def quaternion_to_yaw(self, orientation):
        """ convert quaternion to yaw angle. (converts to vehicle frame)"""
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z*orientation.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    
    def update_global_path(self):
        if self.global_path is None:
            while self.global_path is None:
                self.global_path = self.global_path.plan()

    def update_local_path(self):
        local_goal = self.extract_local_goal()
        self.local_path = self.local_planner.plan(local_goal)

    def publish_local_path(self):
        """ sends the path to waypoint follower """
         # while True:
        #     print("AHAHAHAHA")
        msg = String()
        data = tuple(map(self.Grid.coord_to_pos, self.local_path))
        msg.data = str(data)
        # print(msg.data)
        length = len(data)
        self.waypoint_pub.publish(msg)
        # TODO- MAY NEED TO INTERPOLATE POINTS BEFORE SENDING 
        self.get_logger().info(f"Sent waypoints containing {length} points")


# class def for RRT
class RRT(DualSearch):  # TODO - could make it a child class of dual search node
    """
    have to modify this code skeleton to fit the RRT* algorithm
    current skeleton is better suited for the RRT algorithm

    """
    def __init__(self, am_i_local):
        # super().__init__('rrt_node')
        self.am_i_local = am_i_local
        # TODO - want specific behavior out of the local planner ^^
        self.waypoints = []

        # GRID STUFF -------------------
        self.Grid = None  
        self.mapped=False
        self.tree = []
        self.goal = None # goal for example (CHANGE FOR ACTUAL?)
        self.new_goal=False
        self.goal_tolerance = None
        self.car_width = .4 # meters
        self.path=None
        self.yaw = 0  # initialize yaw for not_allowed_circle
        self.am_local = am_i_local
        
        # ------------------------------

        # MAP TESTING -----_@__!_!_!_#)@#@(_@#(_#@()#))
        with open("data_file.json", 'r') as dt:
            info = dt.read()
            map_info = ast.literal_eval(info)
        dt.close()
        self.Grid = Occupancy(.05, (141, 124), (-1.31, -4.25), map_info)
        self.goal_tolerance = int(0.5/self.Grid.scale)

        # -------------------------------


    def get_iteration_count(self):
        """
        calculates reasonable number of iterations for system 
        based on speed and computational resources
        """
        # TODO - incorporate this later - may also depend on speed of vehicle
        return 100

    def plan(self):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        # pretty sure pose_msg.pose.pose.position.x is required (remove verify later?)
        # print("GETTING POSITION")
        if self.Grid is not None:

            if not self.tree:
                # print("MADE A TREE")
                start = TreeNode(self.coord_x, self.coord_y)
                self.tree.append(start)
            else:
                # print("GETTING CAUGHT HERE")
                if self.no_longer_valid(self.coord_x, self.coord_y) or self.new_goal:
                    self.new_goal=False
                    self.replan(self.coord_x, self.coord_y)
                else:
                    # print("CHOOSING TO GROW TREE")
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
        # TODO - problem does not have to do with the origin
        # TODO - problem also does not have to do with random sampling
        """ expands the current tree"""
        # print(f"{self.goal=}")
        if self.goal is not None:
            # print("doing something?")
            for _ in range(self.get_iteration_count()): # number of iterations
                sampled_point = self.sample()

                #  TODO - JUST FOR FINDING ERRORS 
                # print("still checking for top right")
                # sampled_point =  self.Grid.pos_to_coord(.4674, -3.1547)
                # sampled_point = self.Grid.pos_to_coord(4.4626, -2.1473)

                # print(f"{sampled_point=}")
                # print(f"{self.tree=}")
                nearest_index = self.nearest(self.tree, sampled_point)
                nearest_node = self.tree[nearest_index]
                new_node = self.steer(nearest_node, sampled_point)

                if not self.check_collision(nearest_node, new_node):
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
                    self.waypoints.append(self.Grid.coord_to_pos(new_node))
                    # self.waypoints.append(self.Grid.coord_to_pos((sampled_point[0], sampled_point[1])))
                    self.waypoint_publish()
                    for neighbor in neighbors:
                        cost = new_node.cost + self.line_cost(new_node, neighbor)
                        if cost < neighbor.cost and not self.check_collision(new_node, neighbor):
                            # print("DID SOMETHING")
                            neighbor.parent = new_node
                            neighbor.cost = cost
                    
                    if self.is_goal(new_node, *self.goal):
                        print("found goal?")
                        path = self.find_path(self.tree, new_node)
                        if self.am_local:
                            # first want to get the bars
                            return self.identify_critical_points(ang_tolerance=2, dist_tolerance=1, coords=path)
                        else:
                            return path
                        # should correctly send the path back
                        # self.publish_path(path)
                        # break

    def waypoint_publish(self):
        if self.path is None:
            self.path = self.init_marker(r=1.0)
            
        # print(f"{self.waypoints}")
        x,y = self.waypoints[-1]
        point = Point()
        point.x = x
        point.y = y
        point.z = 0.0
        self.path.points.append(point)

        self.marker_pub.publish(self.path)

    def init_marker(self, r=0.0, g=0.0, b=0.0):
        """ gets the standard settings"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        return marker

    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point
        """
        x = self.Grid.random_point(self.Grid.width) 
        y = self.Grid.random_point(self.Grid.height)

        if not self.in_no_sample_zone(x,y):
            return (x, y)
        return self.sample()
    
    def in_no_sample_zone(self, x, y):
        # print(f"{self.coord_x=} {self.coord_y=}")
        turning_radius = 0.7 / self.Grid.scale  # meters to pixels
        # yaw is zero when in tune with the x-axis, not y axis 
        # print(f"{self.yaw=}")
        yaw = self.yaw +np.pi/2
        left_circle_center = (
            self.coord_x - turning_radius * np.cos(yaw),
            self.coord_y - turning_radius * np.sin(yaw)
        )

        right_circle_center = (
            self.coord_x + turning_radius * np.cos(yaw),
            self.coord_y + turning_radius * np.sin(yaw)
        )
        # print(f"{left_circle_center=}{right_circle_center=}")
        dist_to_left = np.sqrt((x-left_circle_center[0]**2 + (y-left_circle_center[1]**2)))
        dist_to_right = np.sqrt((x-right_circle_center[0])**2 + (y-right_circle_center[1])**2)

        return dist_to_left <= turning_radius or dist_to_right<=turning_radius

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
        # print(f"{direction=}")
        length = LA.norm(direction)
        # print(f"{length=}")
        if length==0.0:
            direction=np.array([0,0])
        else: direction = direction/length
        step_size = int(0.1 / self.Grid.scale)
        new_point = (int(nearest_node.x + (direction[0] * step_size)), int(nearest_node.y + (direction[1] * step_size)))
        # print(f"current point = {nearest_node.x, nearest_node.y}")
        # print(f"{new_point=}")
        # print(f"new point should be {nearest_node.x + direction[0]*step_size}")
        # print(f"goal point = {sampled_point[0], sampled_point[1]}")
        return TreeNode(new_point[0], new_point[1], parent=nearest_node)
    

    def candidate_selection(self, angle, candidates, control_point_dist = 1, num_angles=5):
        """ uses possible points and entry_angle buffer to decide best paths """
        # TODO - NEED TO INTEGRATE THIS WITH OTHER LOCAL BEHAVIOR!! AND TEST THIS HAHAHA
        # TODO - MAKE SURE TO ALSO TEST THE BAND CREATION, DON'T KNOW IF THAT WORKS YET
        p0 = self.coord_x, self.coord_y
        entry_angle = angle
        best_path = None
        best_cost = float('inf')
        best_p1 = best_p2 = None
        angle_buffer = np.radians(5) # current degree buffer in radians
        angle_range = np.linspace(entry_angle-angle_buffer, entry_angle+angle_buffer, num_angles)

        for candidate in candidates:
            for angle in angle_range:
                # create the control points 
                direction_vector = np.array([np.cos(angle), np.sin(angle)])
                p1 = p0 + control_point_dist * direction_vector
                p2 = candidate - control_point_dist * direction_vector

                path = self.bezier_cubic(p0, p1, p2, candidate)

                cost = self.compute_cost(path)
                if cost < best_cost:
                    best_cost = cost
                    best_path = path
                    best_p1, best_p2 = p1, p2
        return best_path, best_cost, best_p1, best_p2

                # now

    def bezier_cubic(self, p0, p1, p2, p3, num_points=50):
        """ creates bezier cubic points based on 4 contact points"""
        t_values = np.linspace(0, 1, num_points)
        curve = []
        for t in t_values:
            point = (1-t)**3 * np.array(p0) + 3*(1-t)**2 * t * np.array(p1) + 3*(1-t)*t**2 * np.array(p2) + t**3*np.array(p3)
            curve.append(point)
        return np.array(curve)

    def compute_cost(self, path):
        """ simple cost estimation based on length and curvature """
        length = np.sum(np.sqrt(np.sum(np.diff(path, axis=0)**2, axis=1)))
        curvature = np.sum(np.abs(np.diff(np.arctan2(np.diff(path[:, 1]), np.diff(path[:, 0])))))
        return length + 10*curvature
    
    

    def angle_within_band(self, desired_angle, band_min, band_max):
        return band_min <= desired_angle <= band_max

    def identify_critical_points(self, ang_tolerance, dist_tolerance, coords):
        """ 
        simplifies the path found from RRT to what is needed 
        
        returns the critical points :)
        """
        # TODO - CAN PROBABLY ALSO EXTRACT HEADING OUT OF THIS (COULD BE USEFUL FOR PATH FINDING)
        # critical_points = [coords[0]]
        critical_points = []
        length = len(coords)
        count=0
        for i in range(1, length-1):
            count+=1
            v1 = np.array([coords[i].x, coords[i].y]) - np.array(coords[i-1].x, coords[i-1].y)
            v2 = np.array(coords[i+1].x, coords[i+1].y) - np.array(coords[i].x, coords[i].y)
            
            angle = self.calculate_angle(v1, v2)
            if angle > np.radians(ang_tolerance) or count==2:
                count=0
                v21 = v2 - v1
                perp = np.linalg.norm(v21)/v21
                bar = set()
                new_pos = TreeNode(coords[i].x, coords[i].y)
                while not self.check_collision(coords[i], new_pos):
                    mult +=1
                    bar.add((new_pos.x, new_pos.y))
                    new_pos.x = int(new_pos.x +perp[0]*mult)
                    new_pos.y = int(new_pos.y +perp[1]*mult)

                while not self.check_collision(coords[i], new_pos):
                    mult -=1
                    bar.add((new_pos.x, new_pos.y))
                    new_pos.x = int(new_pos.x -perp[0]*mult)
                    new_pos.y = int(new_pos.y -perp[1]*mult)
                critical_points.append((coords[i], bar))

            # else:
            #     dist = np.linalg.norm(np.array[coords[i].x, coords[i].y] - np.array(critical_points[-1].x, critical_points[-1].y))
            #     if dist > dist_tolerance:
            #         critical_points.append(coords[i])
            # critical_points.append(coords[-1])

    def calculate_angle(self, v1, v2):
            """ calculates the angle between consecutive points """
            unit_v1 = v1 / np.linalg.norm(v1)
            unit_v2 = v2 / np.linalg.norm(v2)
            dot_product = np.dot(unit_v1, unit_v2)
            return np.arccos(dot_product)

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
        # print(f"{new_pos=}, {near_pos=}")
        stop, start = new_pos, near_pos
        width = self.car_width / self.Grid.scale  # gives us the car width in terms of coordinate squares 
        # assuming new_node is the target (stop) and near_pos is start
        direct_vector = stop-start
        length = int(np.linalg.norm(direct_vector))
        direct_vector = direct_vector / length
        offset = int(width/2)
        square = [np.array([x,y]) for x in range(start[0]-offset, start[0]+(offset+1)) for y in range(start[1]-offset, start[1]+(offset+1))]
        path = set(tuple((array+(mult*direct_vector)).astype(int)) for mult in range(1, length+1) for array in square)
        truthy = any(location in self.Grid for location in path) 
        # if truthy:
        #     print(f"{direct_vector=}")
        return truthy
        
        # TODO - for the more optimized search, (circle mode), can just check IITC, or sensor 
        #   data for soon collision
        # TODO - I also dont think this current method is good for complex maneouvers 
        #   collision checking does feel quite linear... 

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
        assert NotImplementedError

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straight line
        Returns:
            cost (float): the cost value of the line
        """
        # TODO - remember that distances are in terms of pixels
        cost = LA.norm([n1.x - n2.x, n1.y-n2.y])  
        return cost
        # for now cost is just determined by distance

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighbors ([]): neighborhood of nodes as a list of Nodes
        """
        # make radius 1
        radius = int(1 / self.Grid.scale)
        neighbors = []
        for n in tree:
            if LA.norm([node.x - n.x, node.y-n.y]) <= radius:
                # means that the node is "near"
                neighbors.append(n)
        return neighbors
    

    def publish_path(self, path):
        # while True:
        #     print("AHAHAHAHA")
        msg = String()
        data = tuple(map(self.Grid.coord_to_pos, path))
        msg.data = str(data)
        # print(msg.data)
        length = len(data)
        self.waypoint_pub.publish(msg)
        # TODO- MAY NEED TO INTERPOLATE POINTS BEFORE SENDING 
        self.get_logger().info(f"Sent waypoints containing {length} points")
        



def main(args=None):
    rclpy.init(args=args)
    print("RRT Initialized")
    rrt_node = RRT()
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()