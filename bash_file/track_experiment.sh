#!/bin/bash

rosservice call /modquad1/send_waypoint "x: 3.0 
y: 0.0 
z: 0.8" 
echo "Modquad1 go to dock position"

rosservice call /modquad1/track "track_flag: true 
track_traj_initial: true
dock_side: 'back'"
echo "Modquad1 is trying to track"

