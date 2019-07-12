#!/bin/bash

rosservice call /modquad1/send_waypoint "x: 0.0 
y: 0.0 
z: -0.3" 
echo "Modquad1 take off"
read -n 1 -s
rosservice call /modquad1/send_waypoint "x: 0.0 
y: 0.0 
z: 0.0" 
echo "Modquad1 take off"
read -n 1 -s

rosservice call /modquad1/send_waypoint "x: 0.0 
y: 0.0 
z: 0.8" 
echo "Modquad1 go to dock position"
read -n 1 -s

rosservice call /modquad1/track "track_flag: true 
track_traj_initial: true" 
echo "Modquad1 is trying to track"

