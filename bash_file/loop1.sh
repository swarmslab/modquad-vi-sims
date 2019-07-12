#!/bin/bash

rosservice call /modquad1/send_waypoint "x: 2.0 
y: 0.0 
z: 1" 
echo "Modquad1 hover"
read -n 1 -s

rosservice call /modquad1/send_waypoint "x: 2.6 
y: 0.0 
z: 1.0" 
echo "Modquad1 go front"
read -n 1 -s

rosservice call /modquad1/send_waypoint "x: 2.6 
y: 0.5 
z: 1.0" 
echo "Modquad1 go left"
read -n 1 -s

rosservice call /modquad1/send_waypoint "x: 2.0 
y: 0.5 
z: 1.0" 
echo "Modquad1 go back"
read -n 1 -s


rosservice call /modquad1/send_waypoint "x: 2.0 
y: 0.0 
z: 1.0" 
echo "Modquad1 go right"
read -n 1 -s

rosservice call /modquad1/send_waypoint "x: 2.0 
y: 0.0 
z: 0.3" 
echo "Modquad1 go down"
read -n 1 -s
