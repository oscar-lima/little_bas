Blink ros arduino (suscriber)
=============================

This tutorial will help you to try the blink ros arduino tutorial

documentation can be found on:

		http://wiki.ros.org/rosserial_arduino/Tutorials/Blink

1. load hello_world tutorial

		open your arduino IDE
		select file->examples->ros_lib->Blink
		compile and upload the program to your arduino board

2. run the nodes and test

		roscore
		rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
		rostopic pub toggle_led std_msgs/Empty --once

6. Done! now you should be able to see the led 13 toggling every time you publish a message
