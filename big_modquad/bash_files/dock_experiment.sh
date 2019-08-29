#!/bin/bash

rosservice call /modquad2/track "track_flag: true 
dock_side: 'right'
target_ip: 1"
echo "Modquad1 is trying to track"
read -n 1 -s

rosservice call /modquad2/dock "dock_flag: true 
method: 'trajectory'" 
echo "Modquad1 is trying to dock"

