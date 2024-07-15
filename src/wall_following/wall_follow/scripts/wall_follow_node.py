#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        joy_topic = "/joy"
        
        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                                   depth=1,
                                   reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                                   durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)

        self.subscription_joy = self.create_subscription(Joy, joy_topic, self.joy_callback, qos)
        self.subscription_laser = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # TODO: set PID gains
        


        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0 
        self.error = 0.0

        # TODO: store any necessary values you think you'll need
        self.desired_distance = 1.5
        self.velocity = 0.7

        self.kp = 1
        self.ki = 0
        self.kd = .6

        self.safe=False

    def joy_callback(self, msg):
        deadman = 3  # this is the square button 
        if msg.buttons[deadman] ==1:
            # button was pressed
            self.safe=True
        else:
            self.safe=False

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
         # change the angle because it should not be zero :(
        angle_increment = range_data.angle_increment 
        # finds the increment between angle measurements
        angle_min = range_data.angle_min
        index = int((angle - angle_min) / angle_increment)
        range_at_angle = range_data.ranges[index]

        if np.isinf(range_at_angle) or np.isnan(range_at_angle):
            return 10.0  # arbitrarily large value (m) if out of range
        return range_at_angle


    def get_error(self, range_data, desired_dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        angle_1 = 25  # angle to look ahead in degrees
        angle_rad = np.radians(angle_1)
        angle2_rad = np.pi/2


        # get ranges at 0 degrees (front) and theta degrees
        a = self.get_range(range_data, angle_rad )
        b = self.get_range(range_data, angle2_rad)

        theta = angle2_rad - angle_rad

        alpha = np.arctan((a*np.cos(theta)-b)/(a*np.sin(theta)))

        # calculate error
        # alpha = np.arctan2(a * np.cos(angle_rad) - b, a * np.sin(angle_rad))
        AB = b * np.cos(alpha)  # current distance from wall
        AC = AB * np.sin(alpha)  # AC is change in position self.velocity
        CD = AB + AC  # CD represents future position
        error = desired_dist - CD
        return error


    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """

        # PID controller
        self.integral += error
        derivative = (error - self.prev_error)/.1

        control_effort = -self.kp*(error) + self.ki*self.integral + self.kd*derivative
        self.prev_error = error
        self.get_logger().info(f'Calculated Steering Angle: {control_effort}')

        max_steering_angle = np.radians(40)
        control_effort = np.clip(control_effort, -max_steering_angle, max_steering_angle)
        self.get_logger().info(f'Read Error: {error}')
        
        # create and publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = control_effort
        if self.safe:
            drive_msg.drive.speed = 0.7
        else: 
            drive_msg.drive.speed = 0.0

        self.publisher_.publish(drive_msg)


    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        error = self.get_error(msg, self.desired_distance) 
        # self.get_logger().info(f'Current Error distance: {error}')

        self.pid_control(error, self.velocity) 


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()