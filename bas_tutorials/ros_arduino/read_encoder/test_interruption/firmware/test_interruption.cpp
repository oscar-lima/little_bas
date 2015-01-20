//including ros header files
#include <ros.h>
#include <Arduino.h>

int pin = 13;

//all variables which are manipulated by interruption callback
//have to be of type volatile according to arduino documentation
//you can consult it here: 
//http://arduino.cc/en/Reference/attachInterrupt
//look for the "Note" section
volatile int state = HIGH;

ros::NodeHandle nh;

//Note on blink function : using arduino ide actualy it does not matter
//the position of blink function but for ros it gives compilation error
//if you do not put the function before the setup() function
void blink()
{
	//if state is 0 then make it 1 and viceversa
	state = !state;
}

void setup()
{
	//initialize node
	nh.initNode();
	
	//declare pin 13 as output
	pinMode(pin, OUTPUT);
	
	//declare interruption 1 on pin 3 for signal going from low to high with blink callback
	attachInterrupt(1, blink, RISING);
}

void loop()
{
	//write the value of state variable in pin 13
	digitalWrite(pin, state);
	
	//main function would go here...
	//this code will be interrupted when pin 3 says so 
	//just imagine there is a lot of code here and no matter what
	//the arduino will put it on hold and give priority to the interruption
	//...
	
	//hear for callbacks comming from the ros network
	nh.spinOnce();
}
