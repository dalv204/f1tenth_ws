#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan, Joy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import csv

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    
    finds current waypoint! (does it go to it tho?)
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        self.pose_sub = self.create_subscription(PoseStamped, '/current_pose',
                                                 self.pose_callback,
                                                 10)
        
        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                                   depth=1,
                                   reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                                   durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)

        self.subscription_joy = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            qos
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        # for real world testing, would need to add a little safety system
        # also could do a bit more integration between systems 

        self.lookahead_distance = 1.0  # lookahead distance for pure pursuit
        self.waypoints  = self.load_waypoints('waypoints.csv')
        self.current_pose = None

    def subscription_joy(self):
        """function likely used to stop in unsafe scenarios for the real world :)"""
        pass

    def load_waypoints(self, filename):
        """ loads waypoints from the csv """
        waypoints = []
        with open(filename, 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            next(csvreader)
            for row in csvreader:
                x, y, theta = map(float, row)
                waypoints.append((x,y))
        return waypoints

    def pose_callback(self, pose_msg):
        if not self.waypoints:
            return
        self.current_pose = pose_msg.pose

        # find the nearest waypoint in front of the car
        current_position = np.array([self.current_pose.position.x, self.current_pose.position.y])
        distances = [np.linalg.norm(current_position - np.array(wp)) for wp in self.waypoints]

        # find the first waypoint that is ahead of our lookahead distance
        for i, distance in enumerate(distances):
            if distance > self.lookahead_distance:
                goal_waypoint = self.waypoints[i]
                break
        
        # transform goal point to the car's reference point
        goal_x, goal_y = self.transform_goal_point(goal_waypoint)

        # calculate curvature und steering angle 
        curvature = 2 * goal_y / (goal_x**2 + goal_y**2)
        steering_angle = np.arctan(curvature * self.lookahead_distance)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = np.clip(steering_angle, -0.4189, 0.4189)  # limit steering anlge
        drive_msg.drive.speed = 1.0  # set constant speed
        self.drive_pub.publish(drive_msg)

    def transform_goal_point(self, goal_point):
        """ 
        Transform goal point to vehicle reference frame
        """
        # current pose according to global frame
        current_position = np.array([self.current_pose.position.x, self.current_pose.position.y])
        current_orientation = self.current_pose.orientation

        # convert quaternion to yaw angle
        yaw = self.quaternion_to_yaw(current_orientation)

        # transform goal point to the vehicles "POV"
        translation_matrix = np.array([[np.cos(yaw), -np.sin(yaw)],
                                      [np.sin(yaw), np.cos(yaw)]])
        goal_vector = np.array(goal_point) - current_position
        transformed_goal = np.dot(translation_matrix, goal_vector)

        return transformed_goal[0], transformed_goal[1]
    
    def quaternion_to_yaw(self, orientation):
        """ convert quaternion to yaw angle. (converts to vehicle frame)"""
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z*orientation.z)
        return np.arctan2(siny_cosp, cosy_cosp)
        # TODO: find the current waypoint to track using methods mentioned in lecture

        # TODO: calculate curvature/steering angle

        # TODO: publish drive message, don't forget to limit the steering angle.

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
