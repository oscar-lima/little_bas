/*
 * This program aims for testing pwm on arduino motor controller shield
 * at different frequency to reduce/eliminate the acustic noise on the motor
 * 
 * Author: Oscar lima (olima_84@yahoo.com)
 * 
 * Frequency information from:
 * 
 * http://playground.arduino.cc/Code/PwmFrequency
 * http://playground.arduino.cc/Main/TimerPWMCheatsheet (pins 11 and 3 option)
 * http://arduino.cc/en/Tutorial/SecretsOfArduinoPWM
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

void setPwmFrequency(int pin, int divisor) 
{
	// Assuming pins 3 and 11 (arduino motor shield)
	
	byte mode;
	
	switch(divisor) 
	{
		//all frequency calculated values assume 16MHz clock
		case 1: mode = 0x01; break; 	// 31372.55 hz
		case 8: mode = 0x02; break; 	// 3921.16 hz
		case 32: mode = 0x03; break; 	// 980.39 hz
		case 64: mode = 0x04; break; 	// 490.20 hz   <--DEFAULT
		case 128: mode = 0x05; break; 	// 245.10 hz
		case 256: mode = 0x06; break; 	// 122.55 hz
		case 1024: mode = 0x07; break; 	// 30.64 hz
		default: mode = 0x04; break;
	}
	
	//manipulating atmel time register directly:
	//timer 2 set prescale to change PWM frequency
	TCCR2B = TCCR2B & 0b11111000 | mode;
}

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
	digitalWrite(directionA, HIGH);   // turn A clockwise
	digitalWrite(directionB, HIGH);   // turn B clockwise

	// change pwm frequency to avoid noise in the motor
	setPwmFrequency(pwmB, 1); //for this frequency our ear cannot detect the noise anymore
                                  //the drawback is that the h-bridge will heat more and that
                                  //you need a higher duty cycle value to move the motor
	//setPwmFrequency(pwmB, 8);
	//setPwmFrequency(pwmB, 32);
	//setPwmFrequency(pwmB, 64);
	//setPwmFrequency(pwmB, 128);
	//setPwmFrequency(pwmB, 256);
	//setPwmFrequency(pwmB, 1024);
} 

void loop()  
{ 
	// set pwm duty cycle (0-255)
	analogWrite(pwmB, 20);         
	// wait for some time 
	delay(5000);
    
	// set pwm duty cycle (0-255)
	analogWrite(pwmB, 50);         
	// wait for some time 
	delay(5000);
	
	
	// set pwm duty cycle (0-255)
	analogWrite(pwmB, 80);         
	// wait for some time 
	delay(5000);
	
	// set pwm duty cycle (0-255)
	analogWrite(pwmB, 110);         
	// wait for some time 
	delay(5000);
}


