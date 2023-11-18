#!/usr/bin/env python3

import rclpy
import threading
import argparse
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from math import pow, atan2, sqrt, sin, cos, pi

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

    def move2goal(self, goal_pose, goal_theta, distance_tolerance, orientation_tolerance):
        """Moves the turtle to the goal position and orientation."""
        self.get_logger().info(f"Moving towards pose: Position (x: {goal_pose.x}, y: {goal_pose.y}), Orientation: {goal_theta} radians")

        vel_msg = Twist()

        # Move to goal position
        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.angular.z = self.angular_vel(goal_pose)
            self.velocity_publisher.publish(vel_msg)

        # Stop the robot after reaching the goal position
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)
        self.get_logger().info("Goal position reached!")

        # Adjust orientation
        while True:
            orientation_error = goal_theta - self.theta
            orientation_error = atan2(sin(orientation_error), cos(orientation_error))

            if abs(orientation_error) < orientation_tolerance:
                break
            else:
                vel_msg.angular.z = 2 * orientation_error
                self.velocity_publisher.publish(vel_msg)

        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)
        self.get_logger().info("Goal orientation reached!")

def main():
    rclpy.init()

    args_without_ros = rclpy.utilities.remove_ros_args()

    parser = argparse.ArgumentParser(description='Move TurtleBot to a specified pose')
    parser.add_argument('x', type=float, help='Goal x coordinate')
    parser.add_argument('y', type=float, help='Goal y coordinate')
    parser.add_argument('theta', type=float, help='Goal orientation in degrees')
    parser.add_argument('--distance_tolerance', type=float, default=0.1, help='Distance tolerance to the goal')
    parser.add_argument('--orientation_tolerance', type=float, default=5.0, help='Orientation tolerance in degrees')
    parsed_args = parser.parse_args(args_without_ros[1:])

    turtlebot_controller = TurtleBot()

    goal_pose = Point(x=parsed_args.x, y=parsed_args.y)
    goal_theta = parsed_args.theta * (pi / 180)  # Convert to radians
    distance_tolerance = parsed_args.distance_tolerance
    orientation_tolerance = parsed_args.orientation_tolerance * (pi / 180)  # Convert to radians

    move_thread = threading.Thread(target=turtlebot_controller.move2goal, args=(goal_pose, goal_theta, distance_tolerance, orientation_tolerance))
    move_thread.start()

    rclpy.spin(turtlebot_controller)

    move_thread.join()
    turtlebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

