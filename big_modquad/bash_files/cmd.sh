#!/bin/bash

rosservice call /modquad1/send_waypoint "x: -2.0
y: 0.0
z: 1.0
yaw: 0.0"
echo "Modquad1 take off"                       
