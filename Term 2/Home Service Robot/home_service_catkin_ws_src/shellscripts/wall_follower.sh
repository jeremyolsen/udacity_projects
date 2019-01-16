#!/usr/bin/env bash
xterm  -e  " source /opt/ros/kinetic/setup.bash" &
sleep 5
xterm  -e  " export CATKIN_DEVEL_PREFIX=~/catkin_ws/src & roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$CATKIN_DEVEL_PREFIX/world/Blue_U.world  " &
sleep 15
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch  " &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm  -e  " rosrun wall_follower wall_follower"