#!/bin/sh
xterm  -e  " export ROBOT_INITIAL_POSE=\"-x 1 -y 4\"; \
			 roslaunch turtlebot_gazebo turtlebot_world.launch \
			 	world_file:=$(find ~ -type d -name project5_home_service_robot)/map/myworld.world" & 
sleep 10
xterm  -e  " roslaunch turtlebot_navigation amcl_demo.launch \
				map_file:=$(find ~ -type d -name project5_home_service_robot)/map/myworld_map.yaml\
				initial_pose_x:=0 \
				initial_pose_y:=0 \
				" &
sleep 10
# xterm  -e  " roslaunch turtlebot_navigation amcl_demo.launch \
# 				map_file:=$(find ~ -type d -name project5_home_service_robot)/map/mapped_myworld.yaml\
# 				initial_pose_x:=1 \
# 				initial_pose_y:=4 \
# 				" &
# sleep 10
xterm  -e  " rviz -d $(find ~ -type d -name project5_home_service_robot)/rvizConfig/home_service.rviz" &
sleep 10

xterm  -e  " rosrun add_markers add_markers " &
sleep 10

xterm  -e  " rosrun pick_objects pick_objects "
