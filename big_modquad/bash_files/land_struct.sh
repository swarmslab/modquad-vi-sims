#!/bin/bash

rosservice call /modquad1/send_struct_waypoint "Robot_list:
- 101
- 102
x: 0.8
y: 1.0
z: 0.5
yaw: 0.0" 
echo "Structure is landing"
read -n 1 -s                                   

rosservice call /modquad1/send_struct_waypoint "Robot_list:
- 101
- 102
x: 0.8
y: 1.0
z: -0.5
yaw: 0.0" 
echo "Structure is landing"
