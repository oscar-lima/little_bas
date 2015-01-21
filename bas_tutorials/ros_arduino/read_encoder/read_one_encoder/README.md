read_one_encoder
=================

This tutorial focus is on catching all ticks comming from an encoder
by using interruptions.

Keep in mind that for the time being we are not taking care of the
orientation of the motor, this means that it can rotate clockwise or
counterclockwise and the program will still count it as a "tick"

The next tutorial takes care of the direction and adds support for
reading two encoders instead of just one.

Please refer to the following youtube video for a better explanation
regarding this tutorial:

		http://youtu.be/tsG_4G-Gbqw

To compile:

		catkin_make read_one_encoder_firmware
		
To burn the code in the arduino board:

		catkin_make read_one_encoder_firmware_read_one_encoder-upload
		
To test this package:

		connect arduino board to usb
		wire an encoder to pin 2 and 3
		move the motor axis
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
