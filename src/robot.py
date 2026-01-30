import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class rover_Node(Node):

    '''Creates the rover controller node. It subscribes to the joint states
    and publishes velocity commands to the robot.'''
    
    def __init__(self):

        super().__init__('rover_node')

        # Subscriber
        self.subscription = self.create_subscription(
            JointState,
            '/odom',
            self.listener_callback,
            10
        )
        self.subscription

        # Publisher
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10 
        )

        # Loop timer
        self.dt = 0.02  # 50 Hz
        self.timer = self.create_timer(self.dt, self.publisher_callback)

        self.states = None

    def listener_callback(self, msg):
        
        '''Reads the joint states from the subscriber.'''
        self.states = msg.data

    def publisher_callback(self):

        '''Publishes velocity commands to the robot.'''
        if self.states is not None:
            # Create a Twist message with default values
            msg = Twist()
            msg.linear.x = 0.5  # Example linear velocity
            msg.angular.z = 0.1  # Example angular velocity
        else:
            msg = Twist()  # Default empty Twist message if no states are available
        self.publisher.publish(msg)

    def simple_movement(self):

        pass

def main(args=None):

    rclpy.init(args=args)
    rover_node = rover_Node()
    rclpy.spin(rover_node)
    rover_node.destroy_node()
    rclpy.shutdown()  

if __name__ == '__main__':
    main()