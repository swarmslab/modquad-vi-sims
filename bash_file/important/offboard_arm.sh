#!/bin/bash

rosrun mavros mavsys -n mavros101 mode -c OFFBOARD
#rosrun mavros mavsys -n mavros102 mode -c OFFBOARD
echo "Switched to offboard, press ENTER to arm"
read -n 1 -s 

rosrun mavros mavsafety -n mavros101 arm
#rosrun mavros mavsafety -n mavros102 arm
echo "Now armed"

