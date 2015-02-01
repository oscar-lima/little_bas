/*
 * This program implements a proportional speed controller
 * receives a set point and tries to reach it by controlling the pwm
 * in the motor
 * 
 * As expected from control theory the controller does not reach the
 * setpoint, approximately it reaches less than half the requested value
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 */

//******************************************
//Libraries
//******************************************
#include <ros.h>
#include <Arduino.h>

//******************************************
//ROS (robot operating system) related stuff
//******************************************
ros::NodeHandle nh;

//******************************************
//PWM related variables
//******************************************
// arduino motor shield pins definition
int pwmA = 3;
int pwmB = 11;
int directionA = 12;
int directionB = 13;
int brakeA = 9;
int brakeB = 8;
int currentA = 0;
int currentB = 1;
// finish motor pin definition

int pwm_duty_cycle_value = 0;

//*******************************************
//Speedometer related variables
//*******************************************
//interruption #0 related to pin2 in arduino
#define EncoderInterrupt 0 

volatile long encoder_ticks = 0;
unsigned long current_time = 1;
unsigned long delta_time = 1;
unsigned long previous_time = 1;
float angular_speed = 0.0;

//********************************************
//p controller variables
//********************************************
float error = 0;

/*TEST 1
float setpoint = 10;
//speed from now on = angular speed in ticks per quarter second
//proportional gain that overshoots between 18 and 3 speed
//float kp = 30; 
//proportional gain that overshoots between 14 and 1 speed
//float kp = 20;
//stable proportional gain that reaches 4-5 speed
float kp = 10; 
*/

//TEST 2
float setpoint = 20;
//stable proportional gain that reaches 11.5 speed (20 setpoint)
float kp = 10; 

void InterruptA()
{
	//incrementing ticks by one
	encoder_ticks++;
}

void setup()
{
	//initialize node
	nh.initNode();
	
	//initialize serial port at 9600 BAUD speed
	Serial.begin(9600);
	
	//PWM setup
		// initialize pins as input or output accordingly
		pinMode(pwmA, OUTPUT);
		pinMode(pwmB, OUTPUT);     
		pinMode(directionA, OUTPUT);
		pinMode(directionB, OUTPUT);
		pinMode(brakeA, OUTPUT);
		pinMode(brakeB, OUTPUT);
		pinMode(currentA, INPUT);
		pinMode(currentB, INPUT);

		//Disable the brake
		digitalWrite(brakeA, LOW);
		digitalWrite(brakeB, LOW);
		
		//setting pwm frequency to 31372.55 hz
		TCCR2B = TCCR2B & 0b11111000 | 0x01;
		
	//Motor speedometer
		//setup an interruption on pin 2 of arduino uno board with InterruptA callback name
		attachInterrupt(EncoderInterrupt, InterruptA, RISING);
	
}

void loop()
{
	
	//compute motor angular speed in ticks / 0.25 sec
		//reset ticks
		encoder_ticks = 0;
		//getting the current time
		previous_time = millis();
		//wait for ticks
		delay(250);
		//getting current time
		current_time = millis();
		//computing the difference in time (should be around the same value as the delay)
		delta_time = current_time - previous_time;
		//computing average angular speed in ticks per milisecond
		angular_speed = (float)encoder_ticks / (float)delta_time;
		//send speed through serial port
		Serial.print(angular_speed);
		//carriage return for nice output on terminal
		Serial.print("\n");
	
	//pcontroller
		//compute error
		error = setpoint - angular_speed;
		pwm_duty_cycle_value = (int)(kp * error);
		if(pwm_duty_cycle_value > 255) pwm_duty_cycle_value = 255;
                if(pwm_duty_cycle_value < 0) pwm_duty_cycle_value = 0;
		
	//write pwm value in port
	analogWrite(pwmB, pwm_duty_cycle_value);
	
	//hear for callbacks comming from the ros network
	nh.spinOnce();
}
