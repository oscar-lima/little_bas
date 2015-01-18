/*
 * little bas arm driver
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 */

#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>

using namespace std;
using namespace LibSerial;

class BasArm
{
	public:
		/**
		 * @param port_name
		 * example: /dev/ttyUSB0 is the serial usb interface address
		 * 
		 * @param baud
		 * baud rate of the serial port: true 9600, flase 2400
		 * 
		 */
		BasArm(const char *port_name, bool baud);
		
		virtual ~BasArm();
		
		bool init_arm();
		
		void kill_arm();
		
		/**
		 * @param joint
		 * integer telling the servo motor desired number to move
		 * 
		 * @param angle
		 * integer 0-255 value telling the desired angle for the servo
		 * motor to move
		 * 
		 */
		bool move_one_joint(int* joint, int* angle);
		
		/**
		 * @param joint_values
		 * integer array telling the desired servo motor number to move
		 * example : {3, 4, 5} means first move motor 3, then motor 4...
		 * 
		 * @param angle_values
		 * integer array 0-255 value telling the desired angle for each servo
		 * motor to move
		 * example : {60, 80, 10} move motor 3 to 60 value, move motor 4 to 80...
		 * 
		 */
		bool move_many_joints(int* joint_values, int* angle_values);
	
	private:
		bool baud_;
		const char *port_name_;
		SerialStream serial_port_;
};