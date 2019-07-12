#!/bin/bash

rosservice call /modquad2/send_waypoint "x: 0.0 
y: 0.0                                         
z: -0.3 
yaw: 0.0"                                       
echo "Modquad2 take off"                       
read -n 1 -s                                   
                                               
rosservice call /modquad2/send_waypoint "x: 0.0 
y: 0.0                                         
z: 0.5                                        
yaw: 0.0"                                       
echo "Modquad2 go up"                          
read -n 1 -s                                   
                                               
rosservice call /modquad2/send_waypoint "x: 0.0 
y: 0.0                                         
z: 0.5                                          
yaw: 1.5708"                                        
echo "Modquad2 go up"                          
read -n 1 -s                                   
                                               
rosservice call /modquad2/send_waypoint "x: 0.0     
y: 0.0                                        
z: 0.5                                        
yaw: 0.0"                                       
echo "Modquad2 go up"                          
read -n 1 -s                                   
              
rosservice call /modquad2/send_waypoint "x: 0.0
y: 0.0                                         
z: -0.3                                        
yaw: 0.0"                                       
echo "Modquad2 lands"                          
read -n 1 -s                                   
