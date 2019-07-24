#!/bin/bash

rosservice call /modquad0/send_struct_waypoint "Robot_list:
- 0
- 1
x: 0.0
y: 0.0
z: 1.0
yaw: 0.0"
echo "Structure is taking off"
