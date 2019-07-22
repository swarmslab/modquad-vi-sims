#!/bin/bash

#rosservice call /modquad/modquad53/send_waypoint "x: 0.0 
#y: 0.0                                         
#z: 1.2 
#yaw: 0.0"                                       
#echo "Modquad1 take off"                       
#read -n 1 -s                                   
 
rosservice call /modquad/modquad53/send_waypoint "x: 0.1
y: 0.0                                         
z: 1.0 
yaw: 0.0"                                       
echo "Modquad1 take off"                       
read -n 1 -s      
                                              
#rosservice call /modquad/modquad102/send_waypoint "x: 1.2 
#y: 0.0                                         
#z: 1.2 
#yaw: 0.0"                                       
#echo "Modquad2 take off"                       
#read -n 1 -s                                   
                                               
#rosservice call /modquad/modquad53/send_waypoint "x: 0.0
#y: 0.0
#z: 0.4 
#yaw: 0.0"                                        
#echo "Modquad1 go up"                          
#read -n 1 -s                                   
                                               
#rosservice call /modquad/modquad53/send_waypoint "x: 1.2 
#y: 1.5                                         
#z: 1.0 
#yaw: 0.0"                                        
#echo "Modquad2 go up"                          
#read -n 1 -s                                   
                                               
#rosservice call /modquad/modquad53/send_waypoint "x: 0.2
#y: 1.5                                         
#z: 1.5 
#yaw: 0.0"                                        
#echo "Modquad1 go up"                          
#read -n 1 -s                                   
                                               
#rosservice call /modquad/modquad102/send_waypoint "x: 1.2 
#y: 1.5 
#z: 1.5 
#yaw: 0.0" 
#echo "Modquad2 go up"
#read -n 1 -s
