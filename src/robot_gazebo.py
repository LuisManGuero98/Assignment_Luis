import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class rover_Node(Node):

    '''Creates the rover controller node. It subscribes to the joint states
    and publishes velocity commands to the robot.'''
    
    def __init__(self):

        super().__init__('rover_node')

        # Subscriber
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
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

        # Initialize joint states
        self.rover_pose = False

        # Circular trajectory
        self.linear_velocity   = 0.4
        self.angular_velocity  = 0.2

    def odom_callback(self, msg):
        
        '''Reads the joint states from the subscriber.'''
        self.rover_pose = msg.pose.pose
        print(f'Received joint states: {self.rover_pose}')

    def publisher_callback(self):

        '''Publishes velocity commands to the robot.'''
        if self.rover_pose is not None:
            # Create a Twist message with default values
            msg = Twist()
            msg.linear.x = self.linear_velocity  
            msg.angular.z = self.angular_velocity
        else:
            msg = Twist() 
            
        self.publisher.publish(msg)
        print(f'Publishing velocities - Linear: {msg.linear.x}, Angular: {msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = rover_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publisher.publish(Twist()) 
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()