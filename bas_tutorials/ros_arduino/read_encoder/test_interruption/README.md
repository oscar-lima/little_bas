test_interruption
=================

In order to read rotary encoders without losing ticks we need to
read very fast the signals, for that we will use interruptions concept
available in microcontrollers. The idea is to execute a main rutine
and pause or hold the process for a "priority function" which is
triggered by the event of going from low to high value on pins 2,3.

For reading more about interruptions the reader is advised to look
into the arduino documentation in the following link:

		http://playground.arduino.cc/Code/Interrupts

To compile:

		catkin_make test_interruption_firmware
		
To burn the code in the arduino board:

		catkin_make test_interruption_firmware_test_interruption-upload
		
To test this package:

		connect arduino board to usb
		wire a button and connect to pin 3
		push the button and watch for the led 13 blink
		
NOTE: This tutorial does not need roscore or any other node

For further documentation please cat the package.xml file inside this 
folder for reference or alternatively do:

		roscd bas_documentation

and take a look at little bas documentation

thanks!
