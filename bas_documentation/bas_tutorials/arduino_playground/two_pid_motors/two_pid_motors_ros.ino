/*
 * Control two dc motors (PID) for the little_bas differential
 * drive robot through ROS (robot operating system)
 * 
 * We hope that this code can be flexible enough for you to use it in your own robot.
 * 
 * Have a look at the youbutbe channel for little_bas differential drive robot:
 * 
 * 		https://www.youtube.com/channel/UCU-2BdjHIWB2LrqAZn5sVCw
 * 
 * This program assumes you have an arduino motor shield :
 * 
 * 		http://arduino.cc/en/Main/ArduinoMotorShieldR3
 * 
 * We need to wire the motor encoders to the tinker kit molex motor shield pins.
 * For that purpose check on this reference:
 * 
 * http://arduino.stackexchange.com/questions/604/arduino-motor-shield-orange-white-pin-usage
 * 
 * Author  : Oscar Lima (olima_84@yahoo.com)
 * License : GPLv3
 * 
 * How to change the setpoint? : Publish to ROS network:
 * 
 * 		This node subscribes to the following ROS topics:
 * 
 * 		/setpointA  : drives motor A towards that speed
 * 		/setpointB  : drives motor B towards that speed
 * 		/directionA : Changes spinning direction of motor A
 * 		/directionB : CHanges spinning direction of motor B
 * 
 * 		Publishes the following information:
 * 
 * 		arduino_motor_controller/ticksA : long int count of ticks for motor A encoder
 * 		arduino_motor_controller/ticksB : long int count of ticks for motor B encoder
 * 		arduino_motor_controller/pwmA   : last pwm commanded value to motor A
 * 		arduino_motor_controller/pwmB   : last pwm commanded value to motor B
 * 		arduino_motor_controller/speedA : last registered angular speed of motor A (ticks/50ms)
 * 		arduino_motor_controller/speedB : last registered angular speed of motor B (ticks/50ms)
 * 
 */

//******************************************
//Libraries
//******************************************
#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Int8.h>
#include <PID_v1.h>

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
unsigned long current_time = 1;
unsigned long delta_time = 1;
unsigned long previous_time = 1;
float angular_speedA = 0.0;
float angular_speedB = 0.0;

//********************************************
//Encoder interruption functions (for speedometer)
//********************************************
//interruption #0 related to pin2 in arduino
#define EncoderInterruptA 0 
//interruption #1 related to pin3 in arduino
#define EncoderInterruptB 1

//Read a encoder fast enough without losing counts by using interruptions
volatile long encoder_ticksA = 0;
volatile long encoder_ticksB = 0;

//this function will be executed when receiving a change from low to hihg on
//arduino pin 2, (interruption 0)
void InterruptA()
{
	//incrementing ticks in motor A by one
	encoder_ticksA++;
}

//this function will be executed when receiving a change from low to hihg on
//arduino pin 3, (interruption 1)
void InterruptB()
{
	//incrementing ticks in motor B by one
	encoder_ticksB++;
}

//********************************************
//PID controller variables
//********************************************
double inputA, outputA;
double inputB, outputB;
double set_pointA = 0;
double set_pointB = 0;
//setup PID, and controller Kp, Ki, Kd constants
int kp = 6; //6,10,0 also good, a bit slower
int ki = 30;
int kd = 0;
int delay_wait_for_ticks = 50; //sampling time for ticks in ms

PID myPIDA(&inputA, &outputA, &set_pointA, kp, ki, kd, DIRECT);
PID myPIDB(&inputB, &outputB, &set_pointB, kp, ki, kd, DIRECT);

//******************************************
//ROS (robot operating system) related stuff
//******************************************
ros::NodeHandle nh;

//********************************************
//ROS (robot operating system) callbacks
//********************************************
//This functions will be executed when receiving a message on the following topics:
//topics : "setpointA", "setpointB", "directionA", "directionB"
//through the ROS network

//callback triggered when receiving a ros message on set_pointA topic
void setpointACb(const std_msgs::Float64& _setpointA)
{
	set_pointA = _setpointA.data;
}

//callback triggered when receiving a ros message on setpointB topic
void setpointBCb(const std_msgs::Float64& _setpointB)
{
	set_pointB = _setpointB.data;
}

//Motor A change in direction when received a message in "setpointA" topic
void directionACb(const std_msgs::Bool& _directionA)
{
	if(_directionA.data)
	{
		digitalWrite(directionA, HIGH);
	}
	else
	{
		digitalWrite(directionA, LOW);
	}
}

//Motor A change in direction when received a message in "setpointB" topic
void directionBCb(const std_msgs::Bool& _directionB)
{
	if(_directionB.data)
	{
		digitalWrite(directionB, HIGH);
	}
	else
	{
		digitalWrite(directionB, LOW);
	}
}

//Creating ros suscribers to ve able to change the setpoint and motor direction
//from the ROS network by publishing values on the respectice topics
ros::Subscriber<std_msgs::Float64> sub_setpointA("setpointA", &setpointACb);
ros::Subscriber<std_msgs::Float64> sub_setpointB("setpointB", &setpointBCb);
ros::Subscriber<std_msgs::Bool> sub_directionA("directionA", &directionACb);
ros::Subscriber<std_msgs::Bool> sub_directionB("directionB", &directionBCb);

//******************************************
//ROS publishing information
//******************************************
//ticksA_backup, ticksB_backup
std_msgs::UInt64 ticksA_backup;
std_msgs::UInt64 ticksB_backup;
ros::Publisher ticksA_pub("arduino_motor_controller/ticksA", &ticksA_backup);
ros::Publisher ticksB_pub("arduino_motor_controller/ticksB", &ticksB_backup); 

//pwm_duty_cycle_valueA, pwm_duty_cycle_valueB
std_msgs::Int8 pwmA_current_value;
std_msgs::Int8 pwmB_current_value;
ros::Publisher pwmA_pub("arduino_motor_controller/pwmA", &pwmA_current_value);
ros::Publisher pwmB_pub("arduino_motor_controller/pwmB", &pwmB_current_value); 

//angular_speedA, angular_speedB
std_msgs::Int8 speedA_current_value;
std_msgs::Int8 speedB_current_value;
ros::Publisher speedA_pub("arduino_motor_controller/speedA", &speedA_current_value);
ros::Publisher speedB_pub("arduino_motor_controller/speedB", &speedB_current_value);

//******************************************
//Arduino core functions: setup, loop
//******************************************

//Setup function only gets called once (when you turn on or reset the arduino board)
void setup()
{
	//initialize node
	nh.initNode();
	
	//setup the ROS publishers
		nh.advertise(ticksA_pub);
		nh.advertise(ticksB_pub);
		nh.advertise(pwmA_pub);
		nh.advertise(pwmB_pub);
		nh.advertise(speedA_pub);
		nh.advertise(speedB_pub);
	
	//setup the ROS suscribers
		nh.subscribe(sub_setpointA);
		nh.subscribe(sub_setpointB);
		nh.subscribe(sub_directionA);
		nh.subscribe(sub_directionB);
	
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
		
	//setting initial encoder backup data to 0
		ticksA_backup.data = 0;
		ticksB_backup.data = 0;
	
}

//Loop function is like a while(1) for the arduino, will get executed forever
void loop()
{
	
	//compute motor angular speed in ticks / 0.25 sec
		//store a backup of the ticks for publishing to ROS later
		ticksA_backup.data += encoder_ticksA;
		ticksB_backup.data += encoder_ticksB;
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
		
	//write pwm value in arduino port
	analogWrite(pwmA, pwm_duty_cycle_valueA);
	analogWrite(pwmB, pwm_duty_cycle_valueB);
	
	//publish information to the ROS network
		//1. ticksA_backup          2. ticksB_backup
		//3. pwm_duty_cycle_valueA  4. pwm_duty_cycle_valueB
		//5. angular_speedA         6. angular_speedB
		ticksA_pub.publish( &ticksA_backup );
		ticksB_pub.publish( &ticksB_backup );
		pwmA_current_value.data = pwm_duty_cycle_valueA;
		pwmB_current_value.data = pwm_duty_cycle_valueB;
		pwmA_pub.publish( &pwmA_current_value );
		pwmB_pub.publish( &pwmB_current_value );
		speedA_current_value.data = angular_speedA;
		speedB_current_value.data = angular_speedB;
		speedA_pub.publish( &speedA_current_value );
		speedB_pub.publish( &speedA_current_value );
	
	//wait for messages comming from the ros network
	nh.spinOnce();
	//giving some time for ROS to process the information
	delay(10);
}
