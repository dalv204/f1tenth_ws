#!/usr/bin/env python3

""" 
Local Bezier Curve Node
- local operations 

TODO - SHARED PARAMETERS SHOULD BE IN A YAML FILE
TODO - !H - JUST MAKE A __repr__ func for the TreeNode class to show tuple of coords
        -- gonna want to make the KDTree on this side I believe... 
"""
from rclpy import Node
import numpy as np 
from scipy.spatial import KDTree
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from util import TreeNode, Occupancy
import ast

class LBC(Node):
    """ class to process local """
    def __init__(self):
        super().__init__('lbc_node')
        pose_topic = "/ego_racecar/odom"
        scan_topic = "/scan"
        waypoints = "/custom_waypoints"
        global_path_topic = "/global_path"
        map_topic = '/map'

        self.pose_sub_ = self.create_subscription(
            Odometry,
            pose_topic,
            self.pose_callback,
            1
        )

        self.scan_sub_ = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            1
        )
        
        self.global_path_sub_ = self.create_subscription(
            String,
            global_path_topic,
            self.global_path_callback,
            1,
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )


        self.waypoint_pub = self.create_publisher(
            String,
            waypoints,
            10,
        )

        with open("data_file.json", 'r') as dt:
            info = dt.read()
            map_info = ast.literal_eval(info)
        dt.close()

        self.global_path = None
        self.global_path_length = None
        self.kd_tree = None
        self.local_path = None
        self.coord_x, self.coord_y = None, None
        self.dist_tolerance = 0.30 # meters ## depends on the search distance (for nearby in rrty)
        # FOR TESTING ------------------
        self.Grid = Occupancy(.05, (141, 124), (-1.31, -4.25), map_info)
        # ------------------------------
        self.mapped = False

    def pose_callback(self,pose_msg):
        """ updates position and runs localizer (if can) """
        x,y = pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y
        self.coord_x, self.coord_y = self.Grid.pos_to_coord(x,y)
        if self.kd_tree is not None:
            self.update_local_path()
            self.publish_local_path()

    def scan_callback(self, msg):
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

    def global_path_callback(self, msg):
        """ use the path given and construct trees etc """
        self.global_path = tuple(map(lambda x: (np.array(x[0]), x[1], x[2])), ast.literal_eval(msg))
        self.global_path_length = len(self.global_path)
        self.kd_tree = KDTree(item[0] for item in self.global_path)

    def plan(self, point=None):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        # pretty sure pose_msg.pose.pose.position.x is required (remove verify later?)
        print("CALLING PLAN")

        if self.Grid is not None:
            if self.am_local and point is not None:
                priority = point[0]
                # ^^ don't do anything with priority pick for now 
                candidates = point[1]
                entry_angle = point[2]
                return self.candidate_selection(candidates, entry_angle)

                # TODO - should do parallel processing with local and global so no interference
                # TODO - try out once having tested this current alg

    def candidate_selection(self, candidates, angle, control_point_dist = 1, num_angles=5):
        """ uses possible points and entry_angle buffer to decide best paths 
        
        candidates: must be iterable
        angle: must be radians

        """
        # TODO - NEED TO INTEGRATE THIS WITH OTHER LOCAL BEHAVIOR!! AND TEST THIS HAHAHA
        # TODO - MAKE SURE TO ALSO TEST THE BAND CREATION, DON'T KNOW IF THAT WORKS YET
        p0 = self.coord_x, self.coord_y
        best_path = None
        best_cost = float('inf')
        best_p1 = best_p2 = None
        angle_buffer = np.radians(5) # current degree buffer in radians
        angle_range = np.linspace(angle-angle_buffer, angle+angle_buffer, num_angles)

        for candidate in candidates:
            for angle in angle_range:
                # create the control points 
                # TODO - P0 and candidate are both Node objects, so need to do a __sub__ method
                direction_vector = np.array([np.cos(angle), np.sin(angle)])
                shift = control_point_dist * direction_vector
                p1 = p0 + shift
                p2 = candidate - shift

                path = self.bezier_cubic(p0, p1, p2, candidate)

                cost = self.compute_cost(path)
                if cost < best_cost:
                    best_cost = cost
                    best_path = path
                    best_p1, best_p2 = p1, p2

        print(best_cost)
        # best_cost, best_p1, best_p2
        return best_path

    def bezier_cubic(self, p0, p1, p2, p3, num_points=50): ## LOCAL ##
        """ creates bezier cubic points based on 4 contact points"""
        # TODO - !H -- this probably needs some testing to see how to position X and Y
        #       -- also need to figure out how to convert to real coords
        # TODO !H -- different ways to see if a point is invalid...
        #        could make a square and see if any of those points in the OccupancyGrid...
        #        could find closest point and see if it is further than the car's width
        #        implement the square method rn and benchmark later 
        t_values = np.linspace(0, 1, num_points)
        curve = []
        checked = set()
        for t in t_values:
            point = ((1-t)**3 * np.array(p0) + 3*(1-t)**2 * t * np.array(p1) + 3*(1-t)*t**2 * np.array(p2) + t**3*np.array(p3)).astype(int)
            # print("THE POINT HERE IS THAT IT IS ")
            # print(f"{point=}")
            # print("AND THAT IS ALL")
            point_node = TreeNode(point[0], point[1])
            check_point = tuple(point)
            # must be hashable ^^
            if check_point not in checked:
                checked.add(check_point)
                if self.Grid.check_collision(point_node, point_node):
                    return # don't want to get anything from it.. just give up
                curve.append(point)
        return np.array(curve)

    def compute_cost(self, path): ## LOCAL ##
        """ simple cost estimation based on length and curvature """
        length = np.sum(np.sqrt(np.sum(np.diff(path, axis=0)**2, axis=1)))
        curvature = np.sum(np.abs(np.diff(np.arctan2(np.diff(path[:, 1]), np.diff(path[:, 0])))))
        return length + 10*curvature
    

    def angle_within_band(self, desired_angle, band_min, band_max): ## LOCAL ##
        return band_min <= desired_angle <= band_max
    
    def update_local_path(self):
        local_goal = self.extract_local_goal()
        # local path comes in the form of coords (in an array?)
        self.local_path = self.plan(local_goal)

    def extract_local_goal(self):
        """
        uses the consecutive nature of the path to find next position
        """
        # TODO - !L - later change the lookahead distance to be based
        #       on the speed of the vehicle :)

        # TODO - !H - once the testing works normally, 
        # -- use s (straight) and c (curve) 
        # checks if the closest point then looks for the next curve point 
        # and grabs the last point (looks for the next curve and goes back one index)
        lookahead_dist = 1.0 # meters
        lookahead_shift = int(lookahead_dist / self.dist_tolerance)

        current_pose = np.array([self.coord_x, self.coord_y])
        distance, index = self.kd_tree.query(current_pose)
        lookahead_index = (index + lookahead_shift) % self.global_path_length

        # returns the local goal
        return self.global_path[lookahead_index]
    
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
