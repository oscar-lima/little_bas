read_two_encoders
=================

This tutorial is an improvement with respect to read_one_encoder previous
tutorial. Reads two fast encoders trough interruption pins 2 and 3 and 
takes into account the spinning direction (clockwise or counterclockwise)
by reading the signal B of the encoders trough pins 4 and 5.

This program was inspired in information taken from the following website:

		http://www.hessmer.org/blog/2011/01/30/quadrature-encoder-too-fast-for-arduino/

To compile:

		catkin_make read_two_encoders_firmware
		
To burn the code in the arduino board:

		catkin_make read_two_encoders_firmware_read_two_encoders-upload
		
To test this package:

		connect arduino board to usb
		wire two encoders with signals A to pin 2 and 3
		move the motor axis in both directions
		open cutecom terminal (or any other serial terminal of your choice)
		set baud rate to 9600, none parity, for reading only
		disable hexadecimal reading option (to read in int mode)
		select port to open (/dev/arduino/arduino_one_9523335323135181E1B2)
		open port
		watch the numbers comming from the arduino board
		
NOTE: This tutorial does not need roscore or any other node

For further documentation please cat the package.xml file inside this 
folder for reference or alternatively do:

		roscd bas_documentation

and take a look at little bas documentation

thanks!
