ros arduino cmake
=================

This tutorial is a ros tutorial that can be found on:

		http://wiki.ros.org/rosserial_arduino/Tutorials/CMake
		
1. Remarks

- this tutorial only works under ros indigo, i tryed it in hydro and it gives error

- you need to specify the board which you are working with, in my case is "arduino uno"

- if you are reading this tutorial it means that you have cloned the little_bas repository
inside it there is a tutorial folder in which i have putted the code, you can browse for it under:

		~/ros_ws/little_bas/src/little_bas/bas_tutorials/ros_arduino/helloworld

2. compile and upload the code to the arduino board:

		catkin_make helloworld_firmware_hello
		catkin_make helloworld_firmware_hello-upload

3. test the code

		roscore
		rosrun rosserial_python serial_node.py /dev/little_bas/bas_arduino_uno_9523335323135181E1B2
		rostopic echo chatter