#!/bin/bash

rosservice call /modquad0/track "track_flag: true 
dock_side: 'right'
target_ip: 1"
echo "Modquad1 is trying to track"
read -n 1 -s

rosservice call /modquad0/dock "dock_flag: true 
method: 'trajectory'
dock_side: 'right'
target_ip: 1" 
echo "Modquad1 is trying to dock"

