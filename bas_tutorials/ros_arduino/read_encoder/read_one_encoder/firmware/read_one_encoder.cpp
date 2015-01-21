/*
 * This program reads a fast quadratur encoder with an arduino uno board
 * It uses interrupt method to catch all ticks.
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * Please visit the following youtube website for a video explaining in detail this code:
 * 
 * http://youtu.be/tsG_4G-Gbqw
 * 
 */

//including ros header file
#include <ros.h>
#include <Arduino.h>

//Interrupt no. 0 is related to pin2 on the arduino board
#define EncoderInterrupt 0

//Encoder count variable declared as long integer (-32767 to 32767)
volatile long _EncoderTicks = 0;

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
	//write to serial port the value contained in _EncoderTicks variable
	Serial.print(_EncoderTicks);
	
	//write to serial port a carriage return for nice terminal visualization purposes
	Serial.print("\n");
	
	//20 milisecond delay
	delay(20);
	
	//hear for callbacks comming from the ros network
	nh.spinOnce();
}
