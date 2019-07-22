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

TODO: Describe setup for simulation and/or hardware AND usage
