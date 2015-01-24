/*
 * This program reads two fast encoders with orientation (clockwise and ccw)
 * and exposes the count value trough serial port at 9600 BAUD speed.
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * with information from:
 * 
 * reference [1]
 * http://www.hessmer.org/blog/2011/01/30/quadrature-encoder-too-fast-for-arduino/
 * 
 * "It turns out that the regular digitalRead() calls are too slow and bring the 
 * arduino down when I use them in the interrupt routines while the motor runs 
 * at full speed creating more than 40000 encoder ticks per second per motor." [1]
 * 
 * "digitalWriteFast.h library for high performance reads and writes by jrraines
 * see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
 * and http://code.google.com/p/digitalwritefast/
 * " [1]
 * 
 */

// including ros header file
#include <ros.h>
#include <Arduino.h>
#include <digitalWriteFast.h> // read fast pins library by jrraines

// Quadrature encoders
// Left encoder
#define LeftEncoderPinA 2      // interruption pin (high speed)
#define LeftEncoderPinB 4      // B signal of the left encoder connected to normal (not interruption pin)
#define LeftEncoderIsReversed  // used in case the motor are against each other, happens in our case since it is differential drive robot
volatile bool _LeftEncoderBSet;
volatile long _LeftEncoderTicks = 0; // left encoder tick count variable

// Right encoder
#define RightEncoderPinA 3 // interruption pin (high speed)
#define RightEncoderPinB 5 // B signal of the right encoder connected to normal (not interruption pin)
volatile bool _RightEncoderBSet;
volatile long _RightEncoderTicks = 0; // rigth encoder tick count variable

ros::NodeHandle nh;

// Interrupt service routines for the left motor's quadrature encoder
void LeftEncoderInterrupt()
{
  // read the input pin B value
  _LeftEncoderBSet = digitalReadFast(LeftEncoderPinB);   
 
  // adjust counter + if A leads B
  #ifdef LeftEncoderIsReversed
    _LeftEncoderTicks -= _LeftEncoderBSet ? -1 : +1;
  #else
    _LeftEncoderTicks += _LeftEncoderBSet ? -1 : +1;
  #endif
}
 
// Interrupt service routines for the right motor's quadrature encoder
void RightEncoderInterrupt()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  _RightEncoderBSet = digitalReadFast(RightEncoderPinB);   // read the input pin
 
  // increase or decrease count if reversed
  #ifdef RightEncoderIsReversed
	// increase (-*-=+) count by 1 if B value is 1, decrease otherwise
	_RightEncoderTicks -= _RightEncoderBSet ? -1 : +1; //--=+
	/* the previous line is similar to do:
	 * 
	 * if(_RightEncoderBSet == true)
	 * {
	 *     _RightEncoderTicks = _RightEncoderTicks - 1;
	 * }
	 * else
	 * {
	 *     // _RightEncoderBSet is false
	 *     _RightEncoderTicks = _RightEncoderTicks + 1;
	 * }
	 * 
	 * // but condensed in one single line
	 * // condition ? value_if_true : value_if_false
	 */
	
  #else
	// decrease count by -1 if previously B value is 1, increase otherwise
	_RightEncoderTicks += _RightEncoderBSet ? -1 : +1;
  #endif
}

void setup()
{
	// initialize node
	nh.initNode();
	
	// setup serial port at 9600 BAUD speed
	Serial.begin(9600);
	
	// Quadrature encoders
	// Left encoder
	pinMode(LeftEncoderPinA, INPUT);      // sets pin A as input
	pinMode(LeftEncoderPinB, INPUT);      // sets pin B as input
	attachInterrupt(0, LeftEncoderInterrupt, RISING);
	
	// Right encoder
	pinMode(RightEncoderPinA, INPUT);      // sets pin A as input
	pinMode(RightEncoderPinB, INPUT);      // sets pin B as input
	attachInterrupt(1, RightEncoderInterrupt, RISING);
}

void loop()
{
	Serial.print(_LeftEncoderTicks);
	Serial.print("\t");
	Serial.print(_RightEncoderTicks);
	Serial.print("\n");
	
	delay(20);
	
	// hear for callbacks comming from the ros network
	nh.spinOnce();
}
