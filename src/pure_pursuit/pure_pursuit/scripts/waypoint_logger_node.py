#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import numpy as np

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger_node')

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/particle_filter_pose',
            self.pose_callback,
            10
        )

        # open a csv file and log the waypoints
        self.csv_file = open('waypoints.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['x', 'y', 'theta'])

        self.get_logger().info('Waypoint Logger Node has been started.')

    def pose_callback(self, pose_msg):
        # Get the current position and orientation
        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y
        orientation = pose_msg.pose.orientation
        theta = self.quaternion_to_yaw(orientation)

        self.csv_writer.writerow([x,y,theta])
        self.get_logger().info(f'Logged waypoint: x={x}, y={y}, theta={theta}')

    def quaternion_to_yaw(self, orientation):
        """
        Convert quaternion to yaw angle.
        """
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x*orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        return np.arctan2(siny_cosp, cosy_cosp)
    
    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    waypoint_logger_node = WaypointLogger()
    rclpy.spin(waypoint_logger_node)

    waypoint_logger_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()