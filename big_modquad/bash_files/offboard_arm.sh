#!/bin/bash

rosrun mavros mavsys -n modquad0/mavros0 mode -c OFFBOARD
rosrun mavros mavsys -n modquad1/mavros1 mode -c OFFBOARD
rosrun mavros mavsys -n modquad2/mavros2 mode -c OFFBOARD
rosrun mavros mavsys -n modquad3/mavros3 mode -c OFFBOARD
#echo "Switched to offboard, press ENTER to arm"
#read -n 1 -s 

rosrun mavros mavsafety -n modquad0/mavros0 arm
rosrun mavros mavsafety -n modquad1/mavros1 arm
rosrun mavros mavsafety -n modquad2/mavros2 arm
#rosrun mavros mavsafety -n modquad3/mavros3 arm
echo "Now armed"
