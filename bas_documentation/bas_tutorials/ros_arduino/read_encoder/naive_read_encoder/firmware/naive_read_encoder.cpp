/* 
 * Encoder at very low speed example
 * author: Oscar Lima (olima_84@yahoo.com)
 */

#include <ros.h>
#include <std_msgs/Int32.h>

#include <Arduino.h>

ros::NodeHandle nh;

std_msgs::Int32 pushed_msg;
ros::Publisher pub_encoder("encoder_position", &pushed_msg);

const int led_pin = 13;
bool published = true;

int val; 
int encoder0PinA = 3;
int encoder0PinB = 4;

int previous_code = 0;
bool spin_direction = false;
bool skip_error = false;
long encoder0Pos = 0;
bool toggle = false;

void setup()
{
	//initialize node
	nh.initNode();
	//create bool publisher
	nh.advertise(pub_encoder);
	
	//led 13 as output to blink when encoder changes
	pinMode(led_pin, OUTPUT);
	
	//encoder inputs A and B
	pinMode (encoder0PinA,INPUT);
	pinMode (encoder0PinB,INPUT);
	
	//Enable the pullup resistor on the button
	digitalWrite(encoder0PinA, HIGH);
	digitalWrite(encoder0PinB, HIGH);
}

void loop()
{
	bool readingA = digitalRead(encoder0PinA);
	bool readingB = digitalRead(encoder0PinB);
	int code = 0;
	
	//clockwise, 	counterclockwise
	//00, 1			10, 4
	//01, 2			11, 3
	//11, 3			01, 2
	//10, 4			00, 1
	
	//detecting which number is being received (1-4)
	if(readingA == 0 && readingB == 0)
	{
		code = 1;
	}
	else if(readingA == 0 && readingB == 1)
	{
		code = 2;
	}
	else if(readingA == 1 && readingB == 1)
	{
		code = 3;
	}
	else
	{
		code = 4;
	}
	
	if(code != previous_code) //only do something if code has changed
	{
		//detect spin orientation
		int diff = code - previous_code;
		
		switch(diff)
		{
			//clockwise
			case 1: 
			{
				spin_direction = true;
				encoder0Pos++;
			}
			break;
				
			//counterclockwise
			case -1: 
			{
				spin_direction = false;
				encoder0Pos--;
			}
			break;
				
			//counterclockwise
			case 3: 
			{
				spin_direction = false;
				encoder0Pos--;
			}
			break;
				
			//clockwise
			case -3: 
			{
				spin_direction = true;
				encoder0Pos++;
			}
			break;
				
			//error
			default: skip_error = true;
				break;
		}
		
		//toggle pin led
		if(toggle == true)
		{
			toggle = false;
		}
		else
		{
			toggle = true;
		}
		digitalWrite(led_pin, toggle);
		
		//publish encoder value
		pushed_msg.data = encoder0Pos;
		pub_encoder.publish(&pushed_msg);
		
		//updating previous_code value
		previous_code = code;
	}
	
	//reseting skip_error value
	skip_error = false;
	
	//hear for callbacks
	nh.spinOnce();
}