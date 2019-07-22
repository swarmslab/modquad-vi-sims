./bash_files/loop1.sh & ./bash_files/loop2.sh:

These files are just making modquad1 do a square in the air. Nothing special here.

./bash_files/land_both.sh:

Land both modquads INDIVIDUALLY. First sends commands to go to a point in space slightly above the 
ground and then sends a negative z-axis coordinate in order to land.

./bash_files/dock_experiment.sh:

First sends initial waypoint to modquad1 to prepare for docking, then sends service call to 
/modquad1/track to begin tracking (SET track_flag = true, SET track_traj_initial = true), then
sends service call to /modquad1/dock to begin docking process (SET dock_flag = true, SET 
dock_traj_initial = true, all with method of 'trajectory').

./bash_files/loop_struct.sh & ./bash_files/land_struct.sh:

These files concern the scenario where the two quads have already undergone docking and are now 
connected. loop_struct.sh makes the structure (i.e. modquad1 and modquad2 connected to each other) 
do a square in the air. land_struct.sh simply lands the structure. These commands are very similar
to the normal service calls (i.e. when the quads are not connected to each other), but the only 
difference is the addition of a "struct" keyword to the command 
(/modquad1/send_waypoint -> /modquad1/send_struct_waypoint), and providing a Robot_list to
represent the robots in the structure. Once connected, modquad1 is the master of modquad2. This 
suggests that the robot which did the docking action is the master. TODO: CHECK THIS IN CODE

./launch/master_sync.launch:

Synchronizes all the robots in the network. First discovers all robots on the network 
(master_discovery_fkie) and then synchronizes all the topics (master_sync_fkie) with the master
computer. The topics that are synchronized with master computer can be found in 
multimaster_params.yaml

./launch/modquad1.launch:

Launches the docking modQuad module. It synchronizes with the hovering module's odom topic and 
switch_control and control_flag services

./launch/modquad2.launch:

Launches the hovering (waiting) modQuad module. It only synchronizes with the docking module's
odom topic.

./launch/vicon_master_sync.launch:

Synchronizes the master computer and initializes vicon system tracking.

./scripts/plotting/ & ./scripts/testing/:

Directories that have scripts for Guanrui's paper plots and testing. I'm not sure what
everything does. It is most likely completely useless.

./scripts/filter.py:

This is the filtering node for the back tag of the hovering module. Note that 
./misc/testing/filter_back_tag.py and filter.py are exactly the same.


./misc/testing/filter_left_tag.py:

This script is almost exactly the same as filter.py. It only has some simple changes in variables
since it only concerns the left tag of the hovering module. These changes are now 
in ./scripts/filter.py. Simply uncomment the variables for left tag detection and comment the 
previously defined ones.

How PX4 receives docking signal:
First in pos_control.py, the position_control class initializes and then waits for tracking and docking services for both quads. After that, the position_control class does the docking process (after detecting the tag) and when check_dock_state() returns true, position_control calls pub_docked_state_to_px4(true) in modquad_init.py and cancels the services (since the quads are now docked). The method then assigns the coefficients and sends them to both the docking and hovering quads. The message is then intercepted by each modquad_control_cb(), located in 
mavros_extras/src/plugins/modquad_control.cpp, which then calls on modquad_control_send() with all the contents of the message. This gets intercepted by PX4IO::io_set_control_state() in src/drivers/px4io/px4io.cpp which writes the message coefficients into the registers. The coefficients then get picked up by get_control() in MultirotorMixer::mix() in modQuadFirmware/src/lib/mixer/mixer_multirotor.cpp. After they get picked up, the coefficients are factored into the outputs[] matrix that goes to the ESCs and then the motors.

------------------------------------------------------

For developers, the complete path the program takes is:

big_modquad/.../pos_control.py -> 

big_modquad/.../modquad_init.py -> 

mavros_extras/.../modquad_control.cpp ->

For testing: publish mavros_msgs/Cooperative_control msg to /mavros/modquad_control/control_flag
rostopic pub -r 1 /mavros/modquad_control/control_flag mavros_msgs/CooperativeControl  '{x_plusd: 1.0, x_minusd: 1.0, y_plusd: 1.0, y_minusd: 1.0, x: 0.0, y: 0.0, control_flag: true}'

modQuadFirmware/.../px4io.cpp -> 

modQuadFirmware/.../mixer_multirotor.cpp

P.S.: For complete description of modquad_control_msg, check out the last lines of 
mavlink/message_definitions/v1.0/common.xml

------------------------------------------------------

___EXTREMELY IMPORTANT NOTE___

There is a bug in MAVROS where if the RPi's CPU is too overloaded, it will throw the error "CRITICAL NAVIGATION FAILURE - CHECK SENSOR CALIBR". Note that this is NOT a sensor calibration error. Most likely the Raspberry Pi cannot tend to MAVROS' needs on time due to the high BPS. If this error persists, attempt lowering CPU usage either by lowering camera resolution, lowering BPS to PX4, or removing modules from px4_pluginlists.yaml.

