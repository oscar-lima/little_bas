/*
 * code mostly taken from:
 * http://stackoverflow.com/questions/9046649/read-and-write-on-serial-port-in-ubuntu-with-c-c-and-libserial
 * 
 * and:
 * 
 * http://libserial.sourceforge.net/x27.html
 * 
 * adaptations made by: Oscar Lima (olima_84@yahoo.com)
 * 
 * This program write 3 bytes into the serial port /dev/ttyUSB0 usb-serial converter
 * It was tested under ubuntu 14.04
 * 
 * make sure to first install the serial port c++ library by executing the following command:
 * 
 * 		sudo apt-get install libserial-dev
 * 
 */

#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>

using namespace std;
using namespace LibSerial;

int main(int argc, char** argv)
{
	//user must provide 3 argument
	if(argc != 3)
	{
		std::cout << "Usage : ./serial <port number> <0-255 integer angle value>" << std::endl;
		exit(1);
	}

	//creating object of class SerialStream
	SerialStream serial_port;
	
	// Open the serial port
	serial_port.Open("/dev/ttyUSB0");
	
	if (!serial_port.good())
	{
		std::cerr << "Error: Could not open serial port." << std::endl;
		exit(1);
	}

	// Set the baud rate of the serial port
	serial_port.SetBaudRate(SerialStreamBuf::BAUD_9600);
	if (!serial_port.good())
	{
		std::cerr << "Error: Could not set the baud rate." << std::endl;
		exit(1);
	}

	// Set the number of data bits
	serial_port.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
	if (!serial_port.good())
	{
		std::cerr << "Error: Could not set the character size." << std::endl;
		exit(1);
	}

	// Disable parity
	serial_port.SetParity(SerialStreamBuf::PARITY_NONE);
	if (!serial_port.good())
	{
		std::cerr << "Error: Could not disable the parity." << std::endl ;
		exit(1);
	}

	// Set the number of stop bits
	serial_port.SetNumOfStopBits(1);
	if (!serial_port.good())
	{
		std::cerr << "Error: Could not set the number of stop bits." << std::endl;
		exit(1);
	}

	// Turn off hardware flow control
	serial_port.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
	if (!serial_port.good())
	{
		std::cerr << "Error: Could not use hardware flow control." << std::endl;
		exit(1);
	}
	
	//writting to serial port 3 bytes
	const int BUFFER_SIZE = 3;
	char output_buffer[BUFFER_SIZE];

	output_buffer[0] = 255;
	output_buffer[1] = atoi(argv[1]);
	output_buffer[2] = atoi(argv[2]);

	std::cerr << "Writing bytes in serial port /dev/ttyUSB0 : 255, " << atoi(argv[1]) << ", "<< atoi(argv[2]) << std::endl;
	serial_port.write(output_buffer, BUFFER_SIZE);
	
	serial_port.Close();

	return 0;
}