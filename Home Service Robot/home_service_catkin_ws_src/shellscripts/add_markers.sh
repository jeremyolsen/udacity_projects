#!/usr/bin/env bash
xterm  -e  " source /opt/ros/kinetic/setup.bash" &
sleep 5
xterm  -e  " export CATKIN_DEVEL_PREFIX=~/catkin_ws/src & roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$CATKIN_DEVEL_PREFIX/world/Blue_U.world  " &
sleep 15
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$CATKIN_DEVEL_PREFIX/world/Blue_U_Map.yaml " &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 15
xterm  -e  " rosrun add_markers add_markers _home_service_mode:=false "