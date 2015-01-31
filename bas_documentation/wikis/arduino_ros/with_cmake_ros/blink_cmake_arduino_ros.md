blink cmake arduino ros
=======================

This tutorial is a ros tutorial that can be found on:

		http://wiki.ros.org/rosserial_arduino/Tutorials/Blink

The only difference that we add here is that instead of being compiled with arduino IDE
it is compiled by using cmake ros arduino burner interface

2. compile and upload the code to the arduino board:

		catkin_make blink_firmware_blink
		catkin_make blink_firmware_blink-upload

3. test the code

		roscore
		rosrun rosserial_python serial_node.py /dev/little_bas/bas_arduino_uno_9523335323135181E1B2
		rostopic pub toggle_led std_msgs/Empty --once