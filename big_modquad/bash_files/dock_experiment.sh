#!/bin/bash

rosservice call /modquad1/track "track_flag: true 
dock_side: 'forward'
target_ip: 2"
echo "Modquad1 is trying to track"
read -n 1 -s

rosservice call /modquad1/dock "dock_flag: true 
method: 'trajectory'" 
echo "Modquad1 is trying to dock"

