/*
 * This program is an extension for read encoder program, not only reads the ticks
 * comming from the encoder but also computes the angular speed of the motor based
 * on the current time that arduino board provides.
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 */

//including ros header file
#include <ros.h>
#include <Arduino.h>

//Interrupt no. 0 is related to pin2 on the arduino board
#define EncoderInterrupt 0

//Encoder count variable declared as long integer (-32,768 to 32,767)
volatile long _EncoderTicks = 0;
unsigned long current_time = 1;
unsigned long delta_time = 1;
unsigned long previous_time = 1;
float angular_speed = 0.0;

ros::NodeHandle nh;

void InterruptA()
{
	if(_EncoderTicks > 32766)
	{
		//reset to zero the count, avoiding overflow
		_EncoderTicks = 0;
	}
	
	//incrementing ticks by one
	_EncoderTicks++;
}

void setup()
{
	//initialize node
	nh.initNode();
	
	//initialize serial port at 9600 BAUD speed
	Serial.begin(9600);
	
	//setup an interruption on pin 2 of arduino uno board with InterruptA callback name
	attachInterrupt(EncoderInterrupt, InterruptA, RISING);
}

void loop()
{
	//reset ticks
	_EncoderTicks = 0;
	//getting the current time
	previous_time = millis();
	
	//wait for ticks
	//delay(20);
	delay(1000);
	
	//getting current time
	current_time = millis();
	//computing the difference in time (should be around the same value as the delay)
	delta_time = current_time - previous_time;
	//computing average angular speed in ticks per milisecond
	angular_speed = (float)_EncoderTicks / (float)delta_time;
	//write to serial port the speed of the motor V = counts/time
	Serial.print(angular_speed);
	
	//write to serial port a carriage return for nice terminal visualization purposes
	Serial.print("\n");
	
	//keep in mind that function InterruptA() is not explicitely called from loop function
	//but it will be executed based on a change on low to high (RISING) from arduino pin 2
	
	//hear for callbacks comming from the ros network
	nh.spinOnce();
}
