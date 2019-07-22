DOCUMENTATION FOR MODQUAD INSTALLATION ON RPI:

NOTE: This installation guide is not exhaustive. The user is expected to have some Linux knowledge
in order to resolve some of the issues that come up.

Instead of Raspbian, we use Ubuntu MATE 18.04 on our RPi, since this will allow for a more recent
version of ROS, as well as easier access to packages. You can get the Ubuntu MATE image from here:

https://ubuntu-mate.org/raspberry-pi/

You may use Balena-Etcher for Windows/Mac or the dd command on Linux 
(lsblk; sudo dd if=ubuntu-mate-rpi-image-blah-blah.img of=/dev/sdx status=progress)
in order to format your SD card. We use a 32GB sized SD card, and we strongly recommend this size or above.

After the installation, setup your username and password as on a usual desktop. We need to make
a few modifications to the firmware, so open a terminal and type 'sudo raspi-config'. Enable
SSH and the Camera option from "Interfacing Options", and expand the file system. 
From "Interfacing Options" disable the serial login shell and enable the serial port hardware. This is necessary for communicating with Pixhawk. Lastly, add the line 'init_uart_clock=64000000'
to the end of /boot/config.txt. Reboot to save all your changes. After boot, set the ethernet
IPv4 connection settings on your computer from "Automatic" to "Shared to other computers". This 
will allow you to forward the WiFi network to the SSH connection through ethernet.

You may now install ROS Melodic using the standard procedure: http://wiki.ros.org/Installation/Ubuntu. To save on space, install, ros-melodic-desktop and ros-melodic-perception instead of 
ros-melodic-desktop-full. You should then configure the file system, also using the standard 
procedure: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment.

The Pixhawk Firmware should be compiled and uploaded from a laptop instead of the RPi. For 
Pixfalcon, compile with 'make px4fmu-v3_default' and upload the firmware via USB with 
'make px4fmu-v3_default upload'.

To connect Pixfaclon to the RPi, use the TELEM port. RX on RPi -> TX on TELEM (Pixfalcon), TX on RPi -> RX on TELEM (Pixfalcon). Ground can be left alone or connected as well (Ground on RPi -> Ground on Pixfalcon). LEAVE THE TELEM POWER PIN UNCONNECTED. This pin is 5V out, so if you connect it to the RPi it will disrupt your power supply.


(PUT PICTURE HERE OF PIXFALCON RPI CONNECTION!!!!!!!!!!!!!!!!!!!!!!!)


For compiling OpenCV and MAVROS, you should enlarge your swapfile. By default the RPi has a 
small swap partition enabled, so remove it:

'sudo swapoff /swapfile'
'sudo rm /swapfile'

Now make a bigger swapfile. 2GB is good enough:

'sudo fallocate -l 2G /swapfile'
'sudo mkswap /swapfile'
'sudo swapon /swapfile'

Verify your changes with 'free -h'. Before installing OpenCV, make a soft link to Eigen to avoid errors:

'sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen'

Proceed to install OpenCV:

'https://github.com/opencv/opencv'
'cd opencv; mkdir build; cd build'
'cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_TESTS=OFF -DINSTALL_PYTHON_EXAMPLES=OFF -DBUILD_EXAMPLES=OFF -DBUILD_opencv_apps=OFF -DBUILD_DOCS=OFF -DBUILD_PERF_TESTS=OFF ..'
'make -j4; sudo make install -j4'

After that is done, clone the mavros, mavlink, and big_modquad packages inside your src directory
in your catkin workspace. These packages should come from David, as these are customized for
modQuad. Do not clone mavros and mavlink from the official github repos, as they will not work
for modQuad. Proceed to build the workspace with catkin_make or wstool, whichever you prefer.

After the installation is complete, remove the swapfile we have created earlier. This will avoid 
destroying your SD card. Lastly, clone and build the raspicam driver from https://github.com/cedricve/raspicam. Put the RPi ROS camera driver from https://github.com/UbiquityRobotics/raspicam_node in your catkin workspace and rebuild. The setup is now complete.




1. FIGURE OUT GUANRUI'S CODE (USE DAVID'S NOTES)
2. WRITE GAZEBO SIMULATOR FOR MODQUAD

