#!/bin/bash


rosservice call /modquad1/send_waypoint "x: 3.0 
y: 0.0 
z: 0.0" 
echo "Modquad1 land"
read -n 1 -s

rosservice call /modquad1/send_waypoint "x: 3.0 
y: 0.0 
z: -0.5" 
echo "Modquad1 land"

