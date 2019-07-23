# big_modquad
All of the repos for both SITL and actual flying are contained here. Enjoy!

For Gazebo simulations, put these lines in your .bashrc:
{
  source /home/bulgogi/Firmware/Tools/setup_gazebo.bash /path/to/your/Firmware /path/to/your/Firmware/build/px4_sitl_default
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/path/to/your/Firmware
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/path/to/your/Firmware/Tools/sitl_gazebo
} &> /dev/null
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/path/to/your/catkin_ws/build/gazebo_magnet
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/path/to/your/catkin_ws/build/gazebo_magnet
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/your/catkin_ws/src/big_modquad/big_modquad/config/
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:~/ros_ws/src/big_modquad/big_modquad/config/rotors_description/materials/

TODO: Describe setup for simulation and/or hardware AND usage
