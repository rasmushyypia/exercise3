#!/usr/bin/env python3

import rclpy
import threading
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from math import pow, atan2, sqrt, sin, cos


class TurtleBot(Node):

    def __init__(self):
        super().__init__('turtlebot_controller')

        self.pose_subscription = self.create_subscription(Odometry, '/odom', self.update_pose, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.pose = Point()
        self.theta = 0.0
        self.get_logger().info('TurtleBot Node Initialized')


    def update_pose(self, data):
        # Extract current position and yaw angle (theta) from the gazebo simulation
        self.pose = data.pose.pose.position
        self.theta = 2 * atan2(data.pose.pose.orientation.z, data.pose.pose.orientation.w)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
    
    def linear_vel(self, goal_pose, constant=2):
        desired_velocity = constant * self.euclidean_distance(goal_pose)
        max_velocity = 0.5
        return min(desired_velocity, max_velocity)
    
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
    
    def angular_vel(self, goal_pose, constant=0.7):
        goal_theta = self.steering_angle(goal_pose)
        angle_error = goal_theta - self.theta
        
        # Normalize angle to the range [-pi, pi]
        angle_error = atan2(sin(angle_error), cos(angle_error))
        desired_angular_velocity = constant * angle_error

        # Limit the angular velocity to the maximum value
        max_angular_velocity = 2  # You can adjust this value based on your robot's capabilities
        if desired_angular_velocity > 0:
            angular_velocity = min(desired_angular_velocity, max_angular_velocity)
        else:
            angular_velocity = max(desired_angular_velocity, -max_angular_velocity)

        return float(angular_velocity)
    
    def move2goal(self):
        """Moves the turtle to the goal."""
        while rclpy.ok():  # Keep running until ROS is shutdown
            goal_pose = Point()

            # Get the goal position and tolerance from user input
            goal_pose.x = float(input("Set your x goal: "))
            goal_pose.y = float(input("Set your y goal: "))
            distance_tolerance = float(input("Set your tolerance: "))
            self.get_logger().info(f"New goal received: Position (x: {goal_pose.x}, y: {goal_pose.y}) ")

            vel_msg = Twist()
            while self.euclidean_distance(goal_pose) >= distance_tolerance:
                vel_msg.linear.x = self.linear_vel(goal_pose)
                vel_msg.angular.z = self.angular_vel(goal_pose)
                self.velocity_publisher.publish(vel_msg)

            # Stop the robot after reaching the goal
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(vel_msg)

            # Print the message after reaching the goal
            self.get_logger().info("Goal position reached!")




def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtleBot()

    # Start move2goal in a separate thread before spinning
    move_thread = threading.Thread(target=turtlebot_controller.move2goal)
    move_thread.start()

    # This will keep the program running and calling the callback function
    rclpy.spin(turtlebot_controller)

    # Wait for move2goal to finish
    move_thread.join()

    turtlebot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()