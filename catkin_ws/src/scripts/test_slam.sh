#!/bin/bash
set -euxo pipefail

CATKIN_WS="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/../.."
export TURTLEBOT_GAZEBO_WORLD_FILE="$CATKIN_WS/src/world/carlos_big.world"
export TURTLEBOT_GAZEBO_MAP_FILE="$CATKIN_WS/src/map/map.yaml"

xterm  -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm  -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm  -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e "roslaunch turtlebot_teleop keyboard_teleop.launch" &
