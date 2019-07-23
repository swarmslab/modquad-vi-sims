#!/bin/bash

#rosservice call /modquad/modquad222/send_waypoint "x: 0.2 
#y: 1.5 
#z: 1.5
#yaw: 0.0" 
#echo "Modquad1 go to dock position"
#read -n 1 -s

rosservice call /modquad0/track "track_flag: true 
dock_side: 'back'
target_ip: 1"
echo "Modquad1 is trying to track"
read -n 1 -s

rosservice call /modquad0/dock "dock_flag: true 
method: 'trajectory'
dock_side: 'back'
target_ip: 1" 
echo "Modquad1 is trying to dock"

