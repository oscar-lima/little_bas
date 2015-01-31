/*
 * arduino motor shield program
 *
 * Author : Oscar Lima (olima_84@yahoo.com)
 *
 * A program used for testing the arduino motor shield
 *
 * refer to:
 * http://arduino.cc/en/Main/ArduinoMotorShieldR3
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

// the setup routine runs once when you press reset:
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
}

// the loop routine runs over and over again forever:
void loop() 
{
  //spin both motors in one direction
  digitalWrite(pwmA, HIGH);         // turn on motor A
  digitalWrite(pwmB, HIGH);         // turn on motor B
  digitalWrite(directionA, HIGH);   // turn A clockwise
  digitalWrite(directionB, HIGH);   // turn B clockwise
  delay(3000);                      // wait for three seconds
  
  //turn off motors
  digitalWrite(pwmA, LOW);          // turn off motor A
  digitalWrite(pwmB, LOW);          // turn off motor B
  delay(1000);                      // wait for one seconds
  
  //spin both motors in the opposite direction
  digitalWrite(pwmA, HIGH);         // turn on motor A
  digitalWrite(pwmB, HIGH);         // turn on motor B
  digitalWrite(directionA, LOW);    // turn A counterclockwise
  digitalWrite(directionB, LOW);    // turn B counterclockwise
  delay(3000);                      // wait for three seconds
  
  //turn off motors
  digitalWrite(pwmA, LOW);          // turn off motor A
  digitalWrite(pwmB, LOW);          // turn off motor B
  delay(1000);                      // wait for one seconds
}