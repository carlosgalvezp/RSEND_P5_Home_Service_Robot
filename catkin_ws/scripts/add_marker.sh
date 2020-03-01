#!/bin/bash
set -euxo pipefail

CATKIN_WS="$(dirname ${BASH_SOURCE[0]})/.."
source "$CATKIN_WS/devel/setup.bash"

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "roslaunch add_markers add_markers.launch" &
