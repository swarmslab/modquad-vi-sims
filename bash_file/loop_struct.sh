#!/bin/bash

rosservice call /modquad1/send_struct_waypoint "Robot_list:
- 1
- 2
x: 1.0
y: 0.5
z: -0.3" 
echo "Structure take_off"
read -n 1 -s

rosservice call /modquad1/send_struct_waypoint "Robot_list:
- 1
- 2
x: 1.0
y: 0.5
z: 0.5" 
echo "Structure take_off"
read -n 1 -s

rosservice call /modquad1/send_struct_waypoint "Robot_list:
- 1
- 2
x: 1.0
y: 0.5
z: 1.3" 
echo "Structure is going up"
read -n 1 -s

rosservice call /modquad1/send_struct_waypoint "Robot_list:
- 1
- 2
x: 2.2
y: 0.5
z: 2.0" 
echo "Structure is going forward"
read -n 1 -s

rosservice call /modquad1/send_struct_waypoint "Robot_list:
- 1
- 2
x: 2.2
y: 1.5
z: 1.3" 
echo "Structure is going left"
read -n 1 -s

rosservice call /modquad1/send_struct_waypoint "Robot_list:
- 1
- 2
x: 1.0
y: 1.5
z: 2.0" 
echo "Structure is going back"
read -n 1 -s

rosservice call /modquad1/send_struct_waypoint "Robot_list:
- 1
- 2
x: 1.0
y: 0.5
z: 1.3" 
echo "Structure is going right"
read -n 1 -s

rosservice call /modquad1/send_struct_waypoint "Robot_list:
- 101
- 102
x: 1.0
y: 0.5
z: 0.5" 
echo "Structure is landing"
read -n 1 -s

rosservice call /modquad1/send_struct_waypoint "Robot_list:
- 101
- 102
x: 1.0
y: 0.5
z: -0.3" 
echo "Structure is landing"
