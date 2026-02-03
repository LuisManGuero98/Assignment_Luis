# Assignment_Luis
Assignment Luis Manuel

## Gazebo simulation

Simulation from [Space Robotics Gazebo Environments](https://github.com/AndrejOrsula/space_robotics_gz_envs.git)

Run docker container inside the cloned folder with:

```.docker/run.bash gz sim mars.sdf```

ROS2 command to /cmd_vel circle path:

```ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.5}}"``` 

```/model/explorer_r2_sensor_config_1/cmd_vel```

Enter running container:

``` .docker/join.bash```

## ROS Gazebo bridge

To allow ROS2 to visualize Docker Gazebo topics:

```
ros2 run ros_gz_bridge parameter_bridge \
/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
```