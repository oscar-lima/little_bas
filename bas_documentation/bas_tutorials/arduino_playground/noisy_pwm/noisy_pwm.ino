/*
 * This program aims for testing pwm on arduino motor controller shield
 * Since there is no control over pwm frequency, it is possible that the motor
 * will make an acustic noise as it does for my motor.
 * 
 * Author: Oscar lima (olima_84@yahoo.com)
 * 
 * Check the next program to avoid the acustic noise by increasing the pwm
 * frequency value
 * 
 */

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

int pwmValue = 0;

void setup()  
{ 
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

	//set motor rotation
	digitalWrite(directionA, HIGH);   // turn A forward
	digitalWrite(directionB, HIGH);   // turn B forward

} 

void loop()  
{ 
	// set low pwm duty cycle value
	analogWrite(pwmB, 30);         
	// wait for some time 
	delay(5000);
    
	// set medium low pwm duty cycle value
	analogWrite(pwmB, 70);         
	// wait for some time 
	delay(5000);
	
	
	// set medium pwm duty cycle value
	analogWrite(pwmB, 150);         
	// wait for some time 
	delay(5000);
	
	// set maximum pwm duty cycle value
	analogWrite(pwmB, 255);         
	// wait for some time 
	delay(5000);
}


