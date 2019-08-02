#!/bin/bash

rosservice call /modquad0/send_waypoint "x: 0.0
y: 0.0
z: 1.0
yaw: 0.0"
echo "Modquad0 take off"

rosservice call /modquad1/send_waypoint "x: 1.0
y: 0.0
z: 1.0
yaw: 0.0"                                       
echo "Modquad1 take off"                       

rosservice call /modquad2/send_waypoint "x: 2.0
y: 0.0
z: 1.0
yaw: 0.0"
echo "Modquad2 take off"

rosservice call /modquad3/send_waypoint "x: 3.0
y: 0.0
z: 1.0
yaw: 0.0"
echo "Modquad3 take off"

