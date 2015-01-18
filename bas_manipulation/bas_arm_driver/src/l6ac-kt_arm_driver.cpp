/*
 * little bas arm driver
 * 
 * l6ac-kt lynxmotion yellow arm
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 */

#include <l6ac-kt_arm_driver.h>

BasArm::BasArm()
{
	//default values
	port_name_ = "/dev/ttyUSB0";
	baud_ = true;
}

BasArm::BasArm(const char *port_name, bool baud)
{
	port_name_ = port_name;
	baud_ = baud;
}

BasArm::~BasArm()
{
	serial_port_.Close();
}

void BasArm::setup_arm(const char *port_name, bool baud)
{
	port_name_ = port_name;
	baud_ = baud;
}

bool BasArm::init_arm()
{
	// Open the serial port
	serial_port_.Open(port_name_);
	
	if (!serial_port_.good())
	{
		std::cerr << "Error: Could not open serial port." << std::endl;
		return false;
	}

	if(BasArm::baud_)
	{
		serial_port_.SetBaudRate(SerialStreamBuf::BAUD_9600);
		if (!serial_port_.good())
		{
			std::cerr << "Error: Could not set the baud rate." << std::endl;
			return false;
		}
	}
	else
	{
		serial_port_.SetBaudRate(SerialStreamBuf::BAUD_2400);
		if (!serial_port_.good())
		{
			std::cerr << "Error: Could not set the baud rate." << std::endl;
			return false;
		}
	}
	
	// Set the number of data bits
	serial_port_.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
	if (!serial_port_.good())
	{
		std::cerr << "Error: Could not set the character size." << std::endl;
		return false;
	}

	// Disable parity
	serial_port_.SetParity(SerialStreamBuf::PARITY_NONE);
	if (!serial_port_.good())
	{
		std::cerr << "Error: Could not disable the parity." << std::endl ;
		return false;
	}

	// Set the number of stop bits
	serial_port_.SetNumOfStopBits(1);
	if (!serial_port_.good())
	{
		std::cerr << "Error: Could not set the number of stop bits." << std::endl;
		return false;
	}

	// Turn off hardware flow control
	serial_port_.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
	if (!serial_port_.good())
	{
		std::cerr << "Error: Could not use hardware flow control." << std::endl;
		return false;
	}
	
	return true;
}

void BasArm::kill_arm()
{
	serial_port_.Close();
}

bool BasArm::move_one_joint(int* joint, int* angle)
{
	//writting to serial port 3 bytes
	const int BUFFER_SIZE = 3;
	char output_buffer[BUFFER_SIZE];

	output_buffer[0] = 255;
	output_buffer[1] = *joint;
	output_buffer[2] = *angle;
	
	if (serial_port_.good())
	{
		serial_port_.write(output_buffer, BUFFER_SIZE);
		std::cout << "Succesfully written 3 bytes to serial port." << std::endl;
		return true;
	}
	else
	{
		std::cerr << "Error: Could not write bytes to serial port." << std::endl;
		return false;
	}
}

bool BasArm::move_many_joints(int* joint_values, int* angle_values, int number_of_joints)
{
	int array_lenght = 0;
	const int BUFFER_SIZE = 3;
	char output_buffer[BUFFER_SIZE];
	
	for(int i=0; i < number_of_joints; i++)
	{
		output_buffer[0] = 255;
		output_buffer[1] = *joint_values;
		output_buffer[2] = *angle_values;
		
		//writting to serial port 3 bytes
		if (serial_port_.good())
		{
			serial_port_.write(output_buffer, BUFFER_SIZE);
		}
		else
		{
			return false;
		}
		
		joint_values++;
		angle_values++;
		array_lenght++;
	}
	
	std::cout << "Succesfully written " << array_lenght << " bytes to serial port." << std::endl;
	return true;
}
