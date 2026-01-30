# Assignment_Luis
Assignment for PHD possition Luis Manuel

## ROS Gazebo bridge
```ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist```

```ros2 run ros_gz_bridge parameter_bridge /joint_states@sensor_msgs/msg/JointState@gz.msgs.Model```

## Gazebo simulation
```cd /home/luis//Documents/space_robotics_gz_envs```

```.docker/run.bash gz sim mars.sdf```
