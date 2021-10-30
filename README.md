# NavigationRobot

This code when used with ROS (Robot Operating System) and RViz allows a robot
to nagivate in an enviroment.

This project makes use of AI techniques like Bayesian Filtering, A\* Pathfinding.

First, the robot will localise itself to the map using Bayesian Filtering.
Once this has been successful, it will then navigate to a goal using A\* Pathfinding.
Sensors are used to locate obstacles and update the Pathfiding algorithm to avoid
these points.

The robot was able to move around the entire map in a swift and efficient manner.
