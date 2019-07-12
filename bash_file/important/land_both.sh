#!/bin/bash


#rosservice call /modquad/modquad101/send_waypoint "x: 0.8 
#y: 1.0                                         
#z: 0.5"                                        
#echo "Modquad1 land"                           
#read -n 1 -s                                   
                                               
#rosservice call /modquad/modquad102/send_waypoint "x: 2.2 
#y: 1.0                                         
#z: 0.5"                                        
#echo "Modquad2 land"                           
#read -n 1 -s                                   
                                               
rosservice call /modquad/modquad101/send_waypoint "x: 0.0 
y: 0.0                                         
z: -0.5"                                        
echo "Modquad1 land"                           
read -n 1 -s                                   
                                               
rosservice call /modquad/modquad102/send_waypoint "x: 1.2 
y: 0.0                                         
z: -0.5"                                        
echo "Modquad2 land"                           
