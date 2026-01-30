from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='perseverance_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                #[gazebo topic] @ [ROS message type] @ [Gazebo message type]
                # Map the Velocity (Driving) topic
                '/model/nasa_perseverance/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                # Map the Lidar/Scan topic
                '/model/nasa_perseverance/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                # Map the Clock (simulation time)
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            ],
            output='screen'
        )
    ])