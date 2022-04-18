## Dependencies
sudo apt-get install arduino arduino-core ros-<ROS-DISTRO>-rosserial ros-<ROS-DISTRO>-rosserial-arduino

rosrun rosserial_arduino make_libraries.py <arduino library path, execute within the catkin workspace>

or to generate ROS_LIB folder in arduino libraries folder
rosrun rosserial_arduino make_libraries.py .

e.g.
rosrun rosserial_client make_library.py /snap/arduino/current/Arduino/libraries bcr1_control

https://github.com/ros-drivers/rosserial/issues/220
