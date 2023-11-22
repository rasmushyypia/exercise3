import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from math import radians

# "GoGoal" class inherits from the base class "Node"
class DrawSquare(Node):

    def __init__(self):
        # Initialize the node
        super().__init__('GoGoalCmd_publisher')
        
        # Initialize the publisher
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        timer_period = 0.2  # seconds
        # Initialize a timer that excutes call back function every 0.5 seconds
        self.timer = self.create_timer(timer_period, self.drawSquare)
        #self.create_rate(timer_period)

        self.node = rclpy.create_node('simple_node')
        self.node.create_rate(5) 

    def stop_turtlebot(self):
        # define what happens when program is interrupted
        # log that turtlebot is being stopped
        self.get_logger().info('stopping turtlebot')
        # publishing an empty Twist() command sets all velocity components to zero
        # Otherwise turtlebot keeps moving even if command is stopped
        stopper = Twist()
        self.velocity_publisher.publish(stopper)

    def drawSquare(self):

        moveForward = Twist()
        moveForward.linear.x = 0.2

        turnCommand = Twist()
        turnCommand.linear.x = 0.0
        turnCommand.angular.z = radians(45)

        while rclpy.ok():

            self.get_logger().info('moving')
            for x in range (0,20):
                self.velocity_publisher.publish(moveForward)
                rclpy.spin_once(self.node)

            self.get_logger().info('turning')
            for x in range (0,10):
                self.velocity_publisher.publish(turnCommand)
                rclpy.spin_once(self.node)

        # Stopping our robot after the movement is over.
        vel_msg = Twist()
        self.velocity_publisher.publish(vel_msg)

        

        

def main(args=None):
    rclpy.init(args=args)
    
    # we are using try-except tools to  catch keyboard interrupt
    try:
        # create an object for GoForward class
        
        cmd_publisher = DrawSquare()
        # continue untill interrupted
        rclpy.spin(cmd_publisher)
        #cmd_publisher.move2goal()

        #node = DrawSquare()
        #executor = MultiThreadedExecutor()
        #executor.add_node(node)
        #executor.spin()
        
    except KeyboardInterrupt:
        # execute shutdown function
        cmd_publisher.stop_turtlebot()
        # clear the node
        cmd_publisher.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
