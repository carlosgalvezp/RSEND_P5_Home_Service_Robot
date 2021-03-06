#!/bin/bash
set -euxo pipefail

CATKIN_WS="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/../.."
export TURTLEBOT_GAZEBO_WORLD_FILE="$CATKIN_WS/src/world/carlos_big.world"
export TURTLEBOT_GAZEBO_MAP_FILE="$CATKIN_WS/src/map/map.yaml"
source "$CATKIN_WS/devel/setup.bash"

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "rviz --display-config $CATKIN_WS/src/rvizConfig/home_service.rviz" &
sleep 5
xterm -e "roslaunch add_markers add_markers.launch" &
sleep 5
xterm -e "roslaunch pick_objects pick_objects.launch" &
