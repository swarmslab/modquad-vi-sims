# modquad-vi-sims

**Note**: This is still a work in progress. Some of the code may not run well!

All of the repos for both SITL and actual flying are contained here. Enjoy!

For Gazebo simulations, put these lines in your .bashrc:
```
{
  source /path/to/your/Firmware/Tools/setup_gazebo.bash /path/to/your/Firmware /path/to/your/Firmware/build/px4_sitl_default
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/path/to/your/Firmware
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/path/to/your/Firmware/Tools/sitl_gazebo
} &> /dev/null
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/path/to/your/catkin_ws/build/gazebo_magnet
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/path/to/your/catkin_ws/build/gazebo_magnet
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/your/catkin_ws/src/big_modquad/big_modquad/config/
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:/path/to/your/catkin_ws/src/big_modquad/big_modquad/config/rotors_description/materials/
```

In addition, for the simulation you must change whycon's parameters. Go to wherever you installed whycon and edit include/whycon/circle_detector.h. Uncomment the default inner and outer diameters and put ```inner_diameter = 0.087``` and ```outer_diameter = 0.135``` for proper whycon tag distance in Gazebo.

For simulation, make sure to ```git clone https://github.com/yehonathanlitman/Firmware```, ```git checkout experimental-sim``` and then compile the firmware with ```DONT_RUN=1 make px4_sitl_default gazebo```


