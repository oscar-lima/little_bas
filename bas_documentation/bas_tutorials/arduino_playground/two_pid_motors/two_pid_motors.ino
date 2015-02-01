/*
 * Control two dc motors (PID) for the little_bas differential
 * drive robot.
 * 
 * For this tutorial we will need to wire the encoders to the tinker kit
 * pins. For that purpose check on this reference:
 * 
 * http://arduino.stackexchange.com/questions/604/arduino-motor-shield-orange-white-pin-usage
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 */

//******************************************
//Libraries
//******************************************
#include <ros.h>
#include <Arduino.h>
#include <PID_v1.h>

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
int currentA = 0; //current sensor read analog port
int currentB = 1; //current sensor read analog port
// finish motor pin definition

int pwm_duty_cycle_valueA = 0;
int pwm_duty_cycle_valueB = 0;

//*******************************************
//Speedometer related variables
//*******************************************
//interruption #0 related to pin2 in arduino
#define EncoderInterruptA 0 
//interruption #1 related to pin3 in arduino
#define EncoderInterruptB 1

volatile long encoder_ticksA = 0;
volatile long encoder_ticksB = 0;
unsigned long current_time = 1;
unsigned long delta_time = 1;
unsigned long previous_time = 1;
float angular_speedA = 0.0;
float angular_speedB = 0.0;

//********************************************
//PID controller variables
//********************************************
double inputA, outputA;
double inputB, outputB;
double set_pointA = 5;
double set_pointB = 10;
//setup PID, and controller Kp, Ki, Kd constants
int kp = 6; //6,10,0 also good, a bit slower
int ki = 30;
int kd = 0;
int delay_wait_for_ticks = 50; //sampling time for ticks in ms

PID myPIDA(&inputA, &outputA, &set_pointA, kp, ki, kd, DIRECT);
PID myPIDB(&inputB, &outputB, &set_pointB, kp, ki, kd, DIRECT);

void InterruptA()
{
	//incrementing ticks by one
	encoder_ticksA++;
}

void InterruptB()
{
	//incrementing ticks by one
	encoder_ticksB++;
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
		attachInterrupt(EncoderInterruptA, InterruptA, RISING);
		attachInterrupt(EncoderInterruptB, InterruptB, RISING);
	
	//PID controller
		//Enable PID controller (off by default)
		myPIDA.SetMode(AUTOMATIC); //MANUAL = PID off
		myPIDB.SetMode(AUTOMATIC); //MANUAL = PID off
	
}

void loop()
{
	
	//compute motor angular speed in ticks / 0.25 sec
		//reset ticks
		encoder_ticksA = 0;
		encoder_ticksB = 0;
		//getting the current time
		previous_time = millis();
		//wait for ticks
		delay(delay_wait_for_ticks);
		//getting current time
		current_time = millis();
		//computing the difference in time (should be around the same value as the delay)
		delta_time = current_time - previous_time;
		//computing average angular speed in ticks per milisecond
		angular_speedA = (float)encoder_ticksA / (float)delta_time;
		angular_speedB = (float)encoder_ticksB / (float)delta_time;
		//send speed through serial port
		Serial.print(angular_speedA);
		//carriage return for nice output on terminal
		Serial.print("\n");
	
	//PID controller
		inputA = (double)angular_speedA;
		inputB = (double)angular_speedB;
		myPIDA.Compute();
		myPIDB.Compute();
		if(pwm_duty_cycle_valueA > 255) pwm_duty_cycle_valueA = 255;
		if(pwm_duty_cycle_valueB > 255) pwm_duty_cycle_valueB = 255;
		if(pwm_duty_cycle_valueA < 0) pwm_duty_cycle_valueA = 0;
		if(pwm_duty_cycle_valueB < 0) pwm_duty_cycle_valueB = 0;
		pwm_duty_cycle_valueA = outputA;
		pwm_duty_cycle_valueB = outputB;
		
	//write pwm value in port
	analogWrite(pwmA, pwm_duty_cycle_valueA);
	analogWrite(pwmB, pwm_duty_cycle_valueB);
	
	//hear for callbacks comming from the ros network
	nh.spinOnce();
}