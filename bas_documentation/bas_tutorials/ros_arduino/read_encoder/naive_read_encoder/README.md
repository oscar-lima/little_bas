naive_read_encoder
==================

First naive approach for reading a rotary encoder with arduino board

To compile:

		catkin_make naive_read_encoder_firmware
		
To burn the code in the arduino board:

		catkin_make naive_read_encoder_firmware_naive_read_encoder-upload
		
To test this package:

		connect arduino board to usb
		wire encoder properly
		connect signal A and B of encoder to pins 3,4 of the arduino board

		roscore
		rosrun rosserial_python serial_node.py /dev/arduino/arduino_one_9523335323135181E1B2
		rostopic echo /encoder_position

For further documentation please cat the package.xml file inside this 
folder for reference or alternatively do:

		roscd bas_documentation

and take a look at little bas documentation

thanks!
