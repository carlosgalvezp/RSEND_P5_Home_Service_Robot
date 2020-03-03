# RSEND_P5_Home_Service_Robot

This project summarizes the work done in the Robotics Nanodegree Program.

## Goal
The goal is simple: create a home service robot, that performs the following operations:

* Navigate to a given position in the map.
* Pickup an object.
* Navigate to a different position in the map.
* Drop the object there.

Below we explain briefly how this is achieved.

## Mapping
Mapping was performed in Project 4, using the [`rtabmap_ros`](http://wiki.ros.org/rtabmap_ros)
ROS package.

This package creates a map in a SLAM fashion, meaning that the robot needs to localize itself
and build the map at the same time.

SLAM is done based on the robot's sensory data, like a laser scanner and an RGB-D camera.
Loop closure is taken into account to optimize the map and produce consistent results.

After the map is produced, we export it using the `map_server` ROS package, as explained
[here](https://answers.ros.org/question/217097/export-2d-map-from-rviz-andor-rtab-map/).

This produces a `map.pgm` and `map.yaml` file that can be used for localization later on.

We can see this map plotted in Rviz.

## Localization
Localization is performed using the [`amcl`](http://wiki.ros.org/amcl) ROS package.

AMCL stands for Adaptive Monte Carlo Localization. It's core implementation is based on a particle
filter, that fuses information from both control inputs, odometry as well as sensor data,
like the laser scanner and RGB-D camera, to obtain an accurate estimate of the robot's pose
(position and orientation). It's also adaptive in terms of amount of particles at each instant,
increasing computational efficiency - it can use more particles when there's many possible
hypothesis, and few particles when it's confident about the estimate.

In Rviz, we can see the particles as green arrows nearby the turtlebot.

## Navigation
Finally, we use ROS navigation stack to be able to plan paths between different positions
in the map (pick-up and drop-off positions). The ROS navigation stack uses the Dijkstra
algorithm (variant of Uniform Cost Search) to perform path planning.

In Rviz, we can see that the planning algorithm uses a local grid to perform local path
planning, taking into account the shortest path to the goal, as well as inflating
obstacles and giving the extra cost to avoid crashing into them. This grid is continuously
updated based on the robot pose and sensor data.

## Pick objects
The package `pick_objects` that was implemented is rather simple:

* We send a start goal to the `move_base` channel to request the navigation stack
to move the robot there. That will be the pick-up position.
* We wait until the robot has reached the goal.
* Then we sleep 5 seconds to simulate the process of picking up the package.
* Finally, we send a second goal to the navigation stack to get the robot to the drop-off zone.

## Add markers
Finally, the `add_markers` package provides a visual representation of the process of
picking up and dropping objects:

* It subscribes to the `/odometry` topic to get information about where the robot is located
in the map.
* It first displays the object to be picked up at the pick-up zone.
* Then, it waits until the robot arrives to that position, comparing tracking the robot position
from `/odometry` and comparing it to the target position.
* Once the robot arrives to the pick-up position, it hides the marker and waits 5 seconds
to simulate the pick-up process.
* Next, it waits until the robot arrives to the drop-off position, following the same procedure
as before.
* Finally, when the robot arrives to the drop-off position, it displays the same marker
at that position, to show that the robot has dropped off the object.
