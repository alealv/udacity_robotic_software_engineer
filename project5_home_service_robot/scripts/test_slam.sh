#!/bin/sh
xterm  -e  " export ROBOT_INITIAL_POSE=\"-x 1 -y 4\"; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(find ~ -type d -name project5_home_service_robot)/map/myworld.world" & 
sleep 5
xterm  -e  " roslaunch turtlebot_navigation gmapping_demo.launch " & 
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " & 
sleep 5
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch "



# xterm  -e  " export ROBOT_INITIAL_POSE="-x 2"; export TURTLEBOT_STAGE_WORLD_FILE=$(find ~ -type d -name project5_home_service_robot)/map/myworld.world; roslaunch turtlebot_gazebo turtlebot_world.launch" & 