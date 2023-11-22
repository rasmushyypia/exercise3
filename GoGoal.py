import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import rosbag2_py
from rclpy.serialization import serialize_message
from rcl_interfaces.msg import Log


# "GoGoal" class inherits from the base class "Node"
class GoGoal(Node):

    def __init__(self):
        # Initialize the node
        super().__init__('GoGoalCmd_publisher')
        
        velocity_cb_group = ReentrantCallbackGroup()
        pose_cb_group = ReentrantCallbackGroup()
        odom_cb_group = ReentrantCallbackGroup()
        # Initialize the publisher
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10, callback_group=velocity_cb_group)
        
        self.pose_subscriber = self.create_subscription(Pose, '/pose', self.update_pose, 10, callback_group=pose_cb_group)
        
        self.odom_subscriber = self.create_subscription(Odometry, '/odom',self.update_odometry, 10, callback_group=odom_cb_group)
        
        self.pose = Pose()

        timer_period = 0.02  # seconds
        # Initialize a timer that excutes call back function every 0.5 seconds
        self.timer = self.create_timer(timer_period, self.move2goal)
        #self.create_rate(timer_period)

        self.node = rclpy.create_node('simple_node')
        self.node.create_rate(10)
        """
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri='GoGoal_bag',
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)
        topic_info = rosbag2_py._storage.TopicMetadata(
            name='/rosout_agg',
            type='rcl_interfaces/msg/Log',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)
        self.subscription = self.create_subscription(
            Log,
            '/rosout_agg',
            self.logger_callback,
            10)
        self.subscription
    """
    def stop_turtlebot(self):
        # define what happens when program is interrupted
        # log that turtlebot is being stopped
        self.get_logger().info('stopping turtlebot')
        # publishing an empty Twist() command sets all velocity components to zero
        # Otherwise turtlebot keeps moving even if command is stopped
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)
    """  
    def logger_callback(self, msg: Log):
        self.writer.write(
            '/rosout_agg',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)     
    """
    def update_pose(self, data: Pose):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        #self.get_logger().info(str(data.x))
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def update_odometry(self, data: Odometry):
        #self.get_logger().info(str(data.pose.pose.position.x))
        self.pose.x = round(data.pose.pose.position.x,4)
        self.pose.y = round(data.pose.pose.position.y,4)
        self.pose.theta = round(data.pose.pose.orientation.z,4)
        

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=0.25):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=1):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = float(input("Set your x goal: "))
        goal_pose.y = float(input("Set your y goal: "))

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        #distance_tolerance = float(input("Set your tolerance: "))
        distance_tolerance = 0.05

        vel_msg = Twist()
        self.get_logger().info('new goal get, starting turtlebot')

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            rclpy.spin_once(self.node)
            #self.rate.sleep()

            self.get_logger().info('distance left: ' + str(self.euclidean_distance(goal_pose)))          

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)

        self.get_logger().info('goal pose reached')

def main(args=None):
    rclpy.init(args=args)
    
    # we are using try-except tools to  catch keyboard interrupt
    try:
        # create an object for GoForward class
        
        #cmd_publisher = GoGoal()
        # continue untill interrupted
        #rclpy.spin(cmd_publisher)
        #cmd_publisher.move2goal()

        node = GoGoal()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
        
    except KeyboardInterrupt:
        executor.shutdown()
        # execute shutdown function
        node.stop_turtlebot()
        # clear the node
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
