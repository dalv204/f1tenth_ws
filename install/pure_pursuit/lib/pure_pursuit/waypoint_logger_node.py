#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import numpy as np
from nav_msgs.msg import Odometry

class WaypointLogger(Node):
    """
    this class is being coded to suit the simulation, 
    will have to be altered to fit the real vehicle
    
    
    """
    def __init__(self):
        super().__init__('waypoint_logger_node')

        # self.pose_sub = self.create_subscription(
        #     PoseStamped,
        #     '/particle_filter_pose',
        #     self.pose_callback,
        #     10
        # )

        self.sim_pos_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.pose_callback,
            10
        )

        # TODO - would use this in real car ^^

        # open a csv file and log the waypoints
        self.csv_file = open('waypoints_straight.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['x', 'y', 'theta'])

        self.get_logger().info('Waypoint Logger Node has been started.')
        self.last_x = None
        self.last_y = None 
        self.total_distance = 0
        # I only want to log waypoints every half meter for now

    def pose_callback(self, pose_msg):
        # Get the current position and orientation
        # x = pose_msg.pose.position.x
        # y = pose_msg.pose.position.y
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        orientation = pose_msg.pose.pose.orientation
        theta = self.quaternion_to_yaw(orientation)

        # log the waypoint if traveled distance is more than .5 meters
        if self.last_x is None or self.last_y is None:
            self.log_waypoint(x, y, theta)
        else:
            self.total_distance = self.total_distance + np.sqrt((x - self.last_x)**2 + (y-self.last_y)**2)
            if self.total_distance >= 0.8:
                self.log_waypoint(x,y,theta)
                self.total_distance = 0

        self.last_x, self.last_y = x, y
    def log_waypoint(self, x, y, theta):
        self.csv_writer.writerow([x,y,theta])
        self.get_logger().info(f'Logged Waypoint; x={x}, y={y}, theta={theta}')


    def quaternion_to_yaw(self, orientation):
        """
        Convert quaternion to yaw angle.
        """
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x*orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        return np.arctan2(siny_cosp, cosy_cosp)
    
    def destroy_node(self):
        self.csv_file.close()
        print("SAVING CSV")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    print("WAYPOINTS INITIALIZED")
    waypoint_logger_node = WaypointLogger()
    try:
        rclpy.spin(waypoint_logger_node)
    except KeyboardInterrupt:
        print("HEHE")

    waypoint_logger_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()