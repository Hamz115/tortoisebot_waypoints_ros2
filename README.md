### Terminal 1 - Git Checkout & Simulation:
#### source /opt/ros/galactic/setup.bash <br>
#### cd ~/ros2_ws/src/tortoisebot/tortoisebot_waypoints <br>
#### git checkout task2 <br>
#### source ~/ros2_ws/install/setup.bash <br>
#### ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=True <br>
#### Abort and Re-Launch Gazebo if there are any issues. Use kill -9 <gazebo_pid>. <br>

### Terminal 2 - Waypoints Action Server:
#### source /opt/ros/galactic/setup.bash <br>
#### source ~/ros2_ws/install/setup.bash <br>
#### ros2 run tortoisebot_waypoints tortoisebot_action_server <br>

### Terminal 3 - ROS2 Test:
#### source /opt/ros/galactic/setup.bash <br>
#### cd ~/ros2_ws && colcon build && source install/setup.bash <br>
#### colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+ <br>

### For SUCCESS:
#### GOAL_X=0.5 GOAL_Y=0.5 colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+ <br>
### For FAILURE:
#### GOAL_X=1.0 GOAL_Y=2.0 colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+ <br>
