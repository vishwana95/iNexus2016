// Authors: Diunuge Buddhika Wijesinghe
// Competition iNexus 2013
// version: 3.0v: Not Optimized
// Last mod: 28-12-2012
// Comment: updated for finals


#ifndef diuServo_H
#define diuServo_H

#define SERVO_FRONT_GRIP_PIN 7
#define SERVO_FRONT_ELEVATOR_PIN 6

#include <Servo.h>

Servo servoFrontGrip;
Servo servoFrontElevator;

void initializeServo() 
{ 
	servoFrontGrip.attach(SERVO_FRONT_GRIP_PIN);  // attaches the servo on pin 9 to the servo object 
	servoFrontElevator.attach(SERVO_FRONT_ELEVATOR_PIN);

	servoFrontElevator.write(9);    
	delay(50); 
} 

void servoPulse(int servoPin, int angle)
{
  int pulseWidth =(angle * 10) +500; //determines delay
  digitalWrite (servoPin, HIGH); //set servo high
  delayMicroseconds (pulseWidth); //microsecond pause
  digitalWrite(servoPin, LOW); //set servo low
  delay(20); //time to get there
}

void grabFrontBox(){

	servoFrontGrip.attach(SERVO_FRONT_GRIP_PIN); 
	servoFrontGrip.write(180);    
	delay(600); 

	servoFrontGrip.detach();
}

void releaseFrontBox(){
	servoFrontGrip.attach(SERVO_FRONT_GRIP_PIN); 
	int pos=0;

	//if(pos<20)
	//	pos=20;

	servoFrontGrip.write(pos); 
	delay(500); 

	servoFrontGrip.detach();
}

void setHandElevation(int angle){

	servoFrontElevator.attach(SERVO_FRONT_ELEVATOR_PIN);
	servoFrontElevator.write(angle+8);    
	//delay(100); 
}

void releaseElevationServo(){
	servoFrontElevator.detach();
}

void releaseFrontServo(){
	servoFrontGrip.detach();
}

#endif