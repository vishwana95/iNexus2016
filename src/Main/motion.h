// Authors: Diunuge Buddhika Wijesinghe
// Competition iNexus 2013
// version: 2.0v: Not Optimized
// Last mod: 11-11-2012


#ifndef diuMotion_H
#define diuMotion_H


#include "dataTypes.h"
#include "configuration.h"
#include "debug.h"
#include "sensorPanel.h"
#include "status.h"
#include "encoder.h"
#include "diuSonar.h"

/********************* pin naming ***************************************/

//changed for temporary motor driver
#define Motor_Left_Forward  35 	//purple   //35 37
#define Mortor_Left_Reverse 34  //black    //34 36
#define Motor_Left_Enable   10	//orange   //9
#define Motor_Right_Forward 37  //white    //37 35
#define Motor_Right_Reverse 36  //gray     //36 34
#define Motor_Right_Enable  9	//Red      //10

/************************** function prototypes *************************/

//motion control functions
void initializeMotors();
void motorLeft_foward(int pwm);
void motorRight_foward(int pwm);
void motorLeft_reverse(int pwm);
void motorRight_reverse(int pwm);
void motorLeft_stop();
void motorRight_stop();

void moveForward(int leftpwm,int rightpwm);
void turnRight(int pwm);
void turnLeft(int pwm);
void rotateClockwise(int leftpwm,int rightpwm);
void rotateAntiClockwise(int leftpwm,int rightpwm);
void stop();
void stop(int delayms);
void stop_soft();
void reverse(int leftpwm,int rightpwm);
void turn90toLeft();
void turn90toRight();
//void rotateClockwise90();
void rotateClockwise180();
//void rotateAntiClockwise90(int speedLimit=255, int startTime=200);
//void smoothStop();
//void smoothStart();

void gotoNextFrontNode();
void gotoNextFrontNodeSlow();
void gotoNextFrontNodeUnknownDistance();
void turnTowardNode(node currentNode, node nextNode);
boolean turnToDirection(int direction);
void turnTowardAdjacentNode(node currentNode, node adjacentNode);
int  isAdjacent(node node1, node node2);
void setPID();
void calibrateSensors();
void line_following_pd();
void path_follow_PID();
void simple_path_follow();

void path_follow_PID_Sonar();
void path_follow_PID_Arrow();


/************************** function definitions ************************/

void initializeMotors(){
	
	pinMode(Motor_Left_Forward,OUTPUT);
	pinMode(Mortor_Left_Reverse,OUTPUT);
	pinMode(Motor_Right_Forward,OUTPUT);
	pinMode(Motor_Right_Reverse,OUTPUT);

	MIN_RPM = 140;       //120
	MID_RPM = 185;       //155
    MAX_RPM = 255;
	MAX_CORRECTION = 100;

    TEST_RPM = 200;

	totalError = 0;
    previousDeviation = 0;

	lastError =0;

    PID_LeftRPM = 0;
    PID_RightRPM = 0;
}

void setPID(float kp, float ki, float kd){
      
    Kp = kp; //0.2;//0.1
    Ki = ki; //0.0;
    Kd = kd; //4;  //2

}

void setPIDSlow(float kp, float ki, float kd){
      
    Kp_slow = kp; //0.2;//0.1
    Ki_slow = ki; //0.0;
    Kd_slow = kd; //4;  //2

}

void setPIDSlowSlow(float kp, float ki, float kd){
      
    Kp_slow = kp; //0.2;//0.1
    Ki_slow = ki; //0.0;
    Kd_slow = kd; //4;  //2

}

void setReversePID(float kp, float ki, float kd){
     
	Kp_Back = kp; //0.2;//0.1
    Ki_Back = ki; //0.0;
    Kd_Back = kd; //4;  //2
}

void setReversePIDSlow(float kp, float ki, float kd){
     
	Kp_Back_slow = kp; //0.2;//0.1
    Ki_Back_slow = ki; //0.0;
    Kd_Back_slow = kd; //4;  //2
}

void setReversePIDSlowSlow(float kp, float ki, float kd){
     
	Kp_Back_slow_slow = kp; //0.2;//0.1
    Ki_Back_slow_slow = ki; //0.0;
    Kd_Back_slow_slow = kd; //4;  //2
}

void line_following_pd()
{
	position = sensorPannel.readLine(sensors);
	error = position - 4000;
	//error = 0 - error;

	//Serial.println(error);
  
	motorSpeed = Kp * error + Kd * (error - lastError) + Ki * totalError;
	lastError = error;
	totalError += error;

  
	PID_RightRPM = MAX_RPM + motorSpeed;
	PID_LeftRPM = MAX_RPM - motorSpeed;

	if (PID_RightRPM < 0)
		PID_RightRPM = 0;
	if (PID_LeftRPM < 0)
		PID_LeftRPM = 0;
 
	if (PID_RightRPM > 255)
		PID_RightRPM = 255;
    
	if (PID_LeftRPM > 255)
		PID_LeftRPM = 255;
 
	moveForward(PID_LeftRPM, PID_RightRPM);
}

void line_following_pd_slow()
{
	position = sensorPannel.readLine(sensors);
	error = position - 4000;
	//error = 0 - error;

	//Serial.println(error);
  
	motorSpeed = Kp_slow * error + Kd_slow * (error - lastError) + Ki_slow * totalError;
	lastError = error;
	totalError+=error;
 
	PID_RightRPM = MID_RPM + motorSpeed;
	PID_LeftRPM = MID_RPM - motorSpeed;

	if (PID_RightRPM < 0)
		PID_RightRPM = 0;
	if (PID_LeftRPM < 0)
		PID_LeftRPM = 0;
 
	if (PID_RightRPM > 255)
		PID_RightRPM = 255;
    
	if (PID_LeftRPM > 255)
		PID_LeftRPM = 255;
 
	moveForward(PID_LeftRPM, PID_RightRPM);
}

void line_following_pd_slow_slow()
{
	position = sensorPannel.readLine(sensors);
	error = position - 4000;
	//error = 0 - error;

	//Serial.println(error);
  
	motorSpeed = Kp * error + Kd * (error - lastError) + Ki_slow * totalError;
	lastError = error;
	totalError+=error;
 
	PID_RightRPM = 120 + motorSpeed;
	PID_LeftRPM = 120 - motorSpeed;

	if (PID_RightRPM < 0)
		PID_RightRPM = 0;
	if (PID_LeftRPM < 0)
		PID_LeftRPM = 0;
 
	if (PID_RightRPM > 200)
		PID_RightRPM = 200;
    
	if (PID_LeftRPM > 200)
		PID_LeftRPM = 200;
 
	moveForward(PID_LeftRPM, PID_RightRPM);
}

void line_following_reverse_pd()
{
	position = sensorPannelBack.readLine(sensorsBack);
	error = 2500 - position;
	//error = 0 - error;
  
	motorSpeed = Kp_Back * error + Kd_Back* (error - lastError) + Ki_Back*totalError;
	lastError = error;
	totalError += error;
 
  
	PID_RightRPM = MAX_RPM - motorSpeed;
	PID_LeftRPM = MAX_RPM + motorSpeed;

	if (PID_RightRPM < 0)
		PID_RightRPM = 0;
	if (PID_LeftRPM < 0)
		PID_LeftRPM = 0;
 
	if (PID_RightRPM > 255)
		PID_RightRPM = 255;
    
	if (PID_LeftRPM > 255)
		PID_LeftRPM = 255;
 
	//moveForward(PID_LeftRPM, PID_RightRPM);
	reverse(PID_RightRPM, PID_LeftRPM);
}

void line_following_reverse_pd_slow()
{
	position = sensorPannelBack.readLine(sensorsBack);
	error = 2500 - position;
	//error = 0 - error;
  
	motorSpeed = Kp_Back_slow * error + Kd_Back_slow* (error - lastError) + Ki_Back_slow*totalError;
	lastError = error;
	totalError += error;
 
  
	PID_RightRPM = MID_RPM - motorSpeed;
	PID_LeftRPM = MID_RPM + motorSpeed;

	if (PID_RightRPM < 0)
		PID_RightRPM = 0;
	if (PID_LeftRPM < 0)
		PID_LeftRPM = 0;
 
	if (PID_RightRPM > 255)
		PID_RightRPM = 255;
    
	if (PID_LeftRPM > 255)
		PID_LeftRPM = 255;
 
	//moveForward(PID_LeftRPM, PID_RightRPM);
	reverse(PID_RightRPM, PID_LeftRPM);
}

void line_following_reverse_pd_slow_slow()
{
	position = sensorPannelBack.readLine(sensorsBack);
	error = 2500 - position;
	//error = 0 - error;
  
	motorSpeed = Kp_Back_slow_slow * error + Kd_Back_slow_slow* (error - lastError) + Ki_Back_slow_slow*totalError;
	lastError = error;
	totalError += error;
 
  
	PID_RightRPM = 120 - motorSpeed;
	PID_LeftRPM = 120 + motorSpeed;

	if (PID_RightRPM < 60)
		PID_RightRPM = 60;
	if (PID_LeftRPM < 60)
		PID_LeftRPM = 60;
 
	if (PID_RightRPM > 200)
		PID_RightRPM = 200;
    
	if (PID_LeftRPM > 200)
		PID_LeftRPM = 200;
 
	//moveForward(PID_LeftRPM, PID_RightRPM);
	reverse(PID_RightRPM, PID_LeftRPM);
}

void calibrateSensors(){
	Serial.println("Calibrating start..");
	int i;
	#define motionSpeed 150

	int beginEncoderCount = getRighttEncoderCount();

	rotateAntiClockwise(motionSpeed,motionSpeed);
	while((getRighttEncoderCount()-beginEncoderCount)<20)
		sensorPannel.calibrate();

	Serial.println(getRighttEncoderCount()-beginEncoderCount);

	beginEncoderCount = getRighttEncoderCount();
	rotateClockwise(motionSpeed,motionSpeed);
	while((getRighttEncoderCount()-beginEncoderCount)<20)
		sensorPannel.calibrate();

	stop(500);
	Serial.println(getRighttEncoderCount()-beginEncoderCount);

	beginEncoderCount = getRighttEncoderCount();
	rotateClockwise(motionSpeed,motionSpeed);
	while((getRighttEncoderCount()-beginEncoderCount)<20)
		sensorPannel.calibrate();

	Serial.println(getRighttEncoderCount()-beginEncoderCount);

	beginEncoderCount = getRighttEncoderCount();
	rotateAntiClockwise(motionSpeed,motionSpeed);
	while((getRighttEncoderCount()-beginEncoderCount)<20)
		sensorPannel.calibrate();

	stop(2000);
	Serial.println(getRighttEncoderCount()-beginEncoderCount);
	
	Serial.println("Calibrating end..");
}

void path_follow_PID(){

	int kp = 4;
	int kd = 10;

	int Sens3 = Sensor3;
	int Sens4 = Sensor4;
	int Sens5 = Sensor5;
	int Sens6 = Sensor6;
	int Sens7 = Sensor7;

	  //calculate daviation
      if( Sens3==WHITE && Sens4==BLACK && Sens5==BLACK && Sens6==BLACK && Sens7==BLACK)
          deviation = 5;
      if( Sens3==WHITE && Sens4==WHITE && Sens5==BLACK && Sens6==BLACK && Sens7==BLACK)
          deviation = 4;
      if( Sens3==WHITE && Sens4==WHITE && Sens5==WHITE && Sens6==BLACK && Sens7==BLACK)
          deviation = 3;
      if( Sens3==BLACK && Sens4==WHITE && Sens5==BLACK && Sens6==BLACK && Sens7==BLACK)
          deviation = 2;

      if( Sens3==BLACK && Sens4==WHITE && Sens5==WHITE && Sens6==BLACK && Sens7==BLACK)
          deviation = 1;
      if( Sens3==BLACK && Sens4==WHITE && Sens5==WHITE && Sens6==WHITE && Sens7==BLACK)
          deviation = 0;
      if( Sens3==BLACK && Sens4==BLACK && Sens5==WHITE && Sens6==BLACK && Sens7==BLACK)
          deviation = 0;
      if( Sens3==BLACK && Sens4==BLACK && Sens5==WHITE && Sens6==WHITE && Sens7==BLACK)
          deviation = -1;

      if( Sens3==BLACK && Sens4==BLACK && Sens5==BLACK && Sens6==WHITE && Sens7==BLACK)
          deviation = -2;
      if( Sens3==BLACK && Sens4==BLACK && Sens5==WHITE && Sens6==WHITE && Sens7==WHITE)
          deviation = -3;
      if( Sens3==BLACK && Sens4==BLACK && Sens5==BLACK && Sens6==WHITE && Sens7==WHITE)
          deviation = -4;
      if( Sens3==BLACK && Sens4==BLACK && Sens5==BLACK && Sens6==BLACK && Sens7==WHITE)
          deviation = -5;

      correction =  Kp*deviation + Ki*totalError + Kd*(deviation-previousDeviation);
      totalError += correction;
      previousDeviation = deviation;

      PID_LeftRPM = MID_RPM + correction;
      PID_RightRPM = MID_RPM - correction;

      moveForward(PID_LeftRPM, PID_RightRPM);

      //}
      //correction = 0;
      //totalError = 0;
      //sendSensorStatus();
}

void lineFollowPID(){

	int Sens3 = Sensor3;
	int Sens4 = Sensor4;
	int Sens5 = Sensor5;
	int Sens6 = Sensor6;
	int Sens7 = Sensor7;

      	  //calculate daviation
      if( Sens3==WHITE && Sens4==BLACK && Sens5==BLACK && Sens6==BLACK && Sens7==BLACK)
          deviation = 5;
      if( Sens3==WHITE && Sens4==WHITE && Sens5==BLACK && Sens6==BLACK && Sens7==BLACK)
          deviation = 4;
      if( Sens3==WHITE && Sens4==WHITE && Sens5==WHITE && Sens6==BLACK && Sens7==BLACK)
          deviation = 3;
      if( Sens3==BLACK && Sens4==WHITE && Sens5==BLACK && Sens6==BLACK && Sens7==BLACK)
          deviation = 2;

      if( Sens3==BLACK && Sens4==WHITE && Sens5==WHITE && Sens6==BLACK && Sens7==BLACK)
          deviation = 1;
      if( Sens3==BLACK && Sens4==WHITE && Sens5==WHITE && Sens6==WHITE && Sens7==BLACK)
          deviation = 0;
      if( Sens3==BLACK && Sens4==BLACK && Sens5==WHITE && Sens6==BLACK && Sens7==BLACK)
          deviation = 0;
      if( Sens3==BLACK && Sens4==BLACK && Sens5==WHITE && Sens6==WHITE && Sens7==BLACK)
          deviation = -1;

      if( Sens3==BLACK && Sens4==BLACK && Sens5==BLACK && Sens6==WHITE && Sens7==BLACK)
          deviation = -2;
      if( Sens3==BLACK && Sens4==BLACK && Sens5==WHITE && Sens6==WHITE && Sens7==WHITE)
          deviation = -3;
      if( Sens3==BLACK && Sens4==BLACK && Sens5==BLACK && Sens6==WHITE && Sens7==WHITE)
          deviation = -4;
      if( Sens3==BLACK && Sens4==BLACK && Sens5==BLACK && Sens6==BLACK && Sens7==WHITE)
          deviation = -5;

      integral_correction = Ki*totalError;
      if(integral_correction > 100)
            integral_correction = 100;
      if(integral_correction < -100)
            integral_correction = -100;
      
      correction =  Kp*deviation + integral_correction + Kd*(deviation-previousDeviation);
      totalError += correction;
      previousDeviation = deviation;

      if(correction > MAX_CORRECTION)
            correction = MAX_CORRECTION;
      if(correction < -MAX_CORRECTION)
            correction = -MAX_CORRECTION;
            
      PID_LeftRPM = MAX_RPM;
      PID_RightRPM = MAX_RPM + (int)correction;
      
      if(correction > 0)
      {
           PID_LeftRPM  = MAX_RPM - (int)correction;
           PID_RightRPM = MAX_RPM;
      }
      
      /*if(PID_LeftRPM - PID_RightRPM > max_diffrence)
      {
           PID_LeftRPM -= (PID_LeftRPM - PID_RightRPM)/factor_diffrence;
      }
      else if(PID_RightRPM - PID_LeftRPM > max_diffrence)
      {
           PID_RightRPM -= (PID_RightRPM - PID_LeftRPM)/factor_diffrence;
      }*/

      moveForward(PID_LeftRPM,PID_RightRPM);
      
      /*PID_LeftRPM = MID_RPM + (int)correction;
      if(PID_LeftRPM > MAX_RPM)
           PID_LeftRPM = MAX_RPM;

      PID_RightRPM = MID_RPM - (int)correction;
      if(PID_RightRPM > MAX_RPM)
           PID_RightRPM = MAX_RPM;
      moveForward(PID_LeftRPM, PID_RightRPM);*/

}

void simple_path_follow(){
	if(Scout == WHITE){
                  //on the path
                  if( Sensor3==BLACK && Sensor4==WHITE && Sensor5==WHITE && Sensor6==BLACK && Sensor7==BLACK)
                        moveForward(SLOW_PWM,FAST_PWM);
                  else if( Sensor3==BLACK && Sensor4==WHITE && Sensor5==WHITE && Sensor6==WHITE && Sensor7==BLACK)
                        moveForward(FAST_PWM,FAST_PWM);
                  else if( Sensor3==BLACK && Sensor4==BLACK && Sensor5==WHITE && Sensor6==BLACK && Sensor7==BLACK)
                        moveForward(FAST_PWM,FAST_PWM);
                  else if( Sensor3==BLACK && Sensor4==BLACK && Sensor5==WHITE && Sensor6==WHITE && Sensor7==BLACK)
                        moveForward(FAST_PWM,SLOW_PWM);


                  //out of pat; have to turn
                            //turn to left
                  else if( Sensor3==WHITE && Sensor4==BLACK && Sensor5==BLACK && Sensor6==BLACK && Sensor7==BLACK)
                              turnLeft(FAST_PWM);
                  else if( Sensor3==WHITE && Sensor4==WHITE && Sensor5==BLACK && Sensor6==BLACK && Sensor7==BLACK)
                              turnLeft(FAST_PWM);
                  else if( Sensor3==WHITE && Sensor4==WHITE && Sensor5==WHITE && Sensor6==BLACK && Sensor7==BLACK)
                              turnLeft(FAST_PWM);
                  else if( Sensor3==BLACK && Sensor4==WHITE && Sensor5==BLACK && Sensor6==BLACK && Sensor7==BLACK)
                              turnLeft(FAST_PWM);

                             //turn to right
                  else if( Sensor3==BLACK && Sensor4==BLACK && Sensor5==BLACK && Sensor6==WHITE && Sensor7==BLACK)
                              turnRight(FAST_PWM);
                  else if( Sensor3==BLACK && Sensor4==BLACK && Sensor5==WHITE && Sensor6==WHITE && Sensor7==WHITE)
                              turnRight(FAST_PWM);
                  else if( Sensor3==BLACK && Sensor4==BLACK && Sensor5==BLACK && Sensor6==WHITE && Sensor7==WHITE)
                              turnRight(FAST_PWM);
                  else if( Sensor3==BLACK && Sensor4==BLACK && Sensor5==BLACK && Sensor6==BLACK && Sensor7==WHITE)
                              turnRight(FAST_PWM);

				  /*
                  //special
                  else if( Sensor3==WHITE && Sensor4==WHITE && Sensor5==WHITE && Sensor6==WHITE && Sensor7==BLACK)
                        moveForward(FAST_PWM,FAST_PWM);
                  else if( Sensor3==BLACK && Sensor4==WHITE && Sensor5==WHITE && Sensor6==WHITE && Sensor7==WHITE)
                        moveForward(FAST_PWM,FAST_PWM);
                  else if( Sensor1==BLACK && Sensor2==BLACK && Sensor3==BLACK && Sensor4==BLACK && Sensor5==BLACK && Sensor6==BLACK && Sensor7==BLACK && Sensor8==BLACK && Sensor9==BLACK)
                        moveForward(255,255);
				  */

            else if(isAllBlack()){
                  // all black
                  stop();
				  Serial.write("Stopping.. ");
                  //sendSensorStatus();
				  delay(1000);
            }
            else{
                  stop();
                  //reverse(220,220);
				  Serial.write("Stopping.. ");
                  //sendSensorStatus();
                  delay(1000);
            }
      }
	else{
		stop();
			Serial.write("Stopping.. ");
            //sendSensorStatus();
			delay(1000);
	}
}





void path_follow_PID_Sonar(){

	int sonarFrontDis = getFrontSonarReading();
	
	int sonarRightDis = getRightSonarReading();
	int sonarLeftDis = getLeftSonarReading();

	deviation =  sonarLeftDis - sonarRightDis;
	Serial.write("Deviation :");
	Serial.println(deviation);

	integral_correction = Ki*totalError;
	if(integral_correction > 50)
		integral_correction = 50;
	if(integral_correction < -50)
		integral_correction = -50;

	correction =  Kp*deviation + integral_correction + Kd*(deviation-previousDeviation);
	totalError += correction;
	previousDeviation = deviation;

	if(correction > 50)
		correction = 50;
	if(correction < -50)
		correction = -50;

	PID_LeftRPM = 150;
	PID_RightRPM = 150 + (int)correction;

	if(correction > 0)
	{
		PID_LeftRPM  = 150 - (int)correction;
		PID_RightRPM = 150;
	}

	/*if(PID_LeftRPM - PID_RightRPM > max_diffrence)
	{
	PID_LeftRPM -= (PID_LeftRPM - PID_RightRPM)/factor_diffrence;
	}
	else if(PID_RightRPM - PID_LeftRPM > max_diffrence)
	{
	PID_RightRPM -= (PID_RightRPM - PID_LeftRPM)/factor_diffrence;
	}*/


	if(PID_LeftRPM > 150)
		PID_LeftRPM = 150;

	if(PID_RightRPM > 150)
		PID_RightRPM = 150;

	Serial.write("\tRpm :( ");
	Serial.print(PID_LeftRPM);
	Serial.write(" , ");	
	Serial.print(PID_RightRPM);
	Serial.println(" )");	

	moveForward(PID_LeftRPM,PID_RightRPM);
}

void path_follow_PID_Arrow(){


	position = sensorPannel.readLine(sensors);
	error = (int)position - 3500;
	Serial.write("\tError :( ");
	Serial.print(error);
  
	int motorSpeedCorrection = Kp_slow * error + Kd_slow* (error - lastError) + Ki_slow*totalError;
	lastError = error;
	totalError += error;

	if(motorSpeedCorrection > 50)
		motorSpeedCorrection = 50;
	if(motorSpeedCorrection < -50)
		motorSpeedCorrection = -50;

	PID_LeftRPM = 150;
	PID_RightRPM = 150 + (int)motorSpeedCorrection;

	if(correction > 0)
	{
		PID_LeftRPM  = 150 - (int)motorSpeedCorrection;
		PID_RightRPM = 150;
	}


	//if(PID_LeftRPM > 150)
	//	PID_LeftRPM = 150;

	//if(PID_RightRPM > 150)
	//	PID_RightRPM = 150;

	Serial.write("\tRpm :( ");
	Serial.print(PID_LeftRPM);
	Serial.write(" , ");	
	Serial.print(PID_RightRPM);
	Serial.println(" )");	

	moveForward(PID_LeftRPM,PID_RightRPM);
}


//motion control functions

void motorLeft_foward(int leftpwm){
    digitalWrite(Motor_Left_Forward,HIGH);
	digitalWrite(Mortor_Left_Reverse,LOW);
	analogWrite(Motor_Left_Enable,leftpwm);
}

void motorLeft_reverse(int leftpwm){
    digitalWrite(Motor_Left_Forward,LOW);
	digitalWrite(Mortor_Left_Reverse,HIGH);
	analogWrite(Motor_Left_Enable,leftpwm);
}


void motorRight_foward(int rightpwm){
	digitalWrite(Motor_Right_Forward,HIGH);
	digitalWrite(Motor_Right_Reverse,LOW);
	analogWrite(Motor_Right_Enable,rightpwm);
}

void motorRight_reverse(int rightpwm){
	digitalWrite(Motor_Right_Forward,LOW);
	digitalWrite(Motor_Right_Reverse,HIGH);
	analogWrite(Motor_Right_Enable,rightpwm);
}

void motorLeft_stop(){
	//analogWrite(Motor_Left_Enable,255);
	digitalWrite(Motor_Left_Forward,HIGH);
	digitalWrite(Mortor_Left_Reverse,HIGH);
}

void motorRight_stop(){
	//analogWrite(Motor_Right_Enable,255);
	digitalWrite(Motor_Right_Forward,HIGH);
	digitalWrite(Motor_Right_Reverse,HIGH);
}


void moveForward(int leftpwm,int rightpwm){
	motorLeft_foward(leftpwm);
    motorRight_foward(rightpwm);
}

void reverse(int leftpwm,int rightpwm){
	motorLeft_reverse(leftpwm);
	motorRight_reverse(rightpwm);
}

void turnRight(int pwm){
	motorLeft_foward(pwm);
	motorRight_stop();
}

void turnLeft(int pwm){
	motorRight_foward(pwm);
	motorLeft_stop();
}

void rotateClockwise(int leftpwm,int rightpwm){
	motorLeft_foward(leftpwm);
	motorRight_reverse(rightpwm);
}


void rotateAntiClockwise(int leftpwm,int rightpwm){
	motorLeft_reverse(leftpwm);
	motorRight_foward(rightpwm);
}

void stop(){
	motorLeft_stop();
    motorRight_stop();
	analogWrite(Motor_Left_Enable,255);
	analogWrite(Motor_Right_Enable,255);
}

void stop(int delayms){
	motorLeft_stop();
    motorRight_stop();
	analogWrite(Motor_Left_Enable,255);
	analogWrite(Motor_Right_Enable,255);
	delay(delayms);
}

void stop_soft(){
	motorLeft_stop();
    motorRight_stop();
	for(int pwm=0; pwm<256; pwm++){
		delayMicroseconds(1);
		analogWrite(Motor_Left_Enable,pwm);
		analogWrite(Motor_Right_Enable,pwm);
	}
}

void smoothStop(int stopTime=200){
	//int stopTime=200;
	motorLeft_stop();
    motorRight_stop();
	for(int pwm=0; pwm<256; pwm++){
		analogWrite(Motor_Left_Enable,pwm);
		analogWrite(Motor_Right_Enable,pwm);
		delayMicroseconds(stopTime*1000/256);
		//Serial.println(pwm);
	}
}

void smoothStart(int startTime=100, int speedLimit=255){

	moveForward(120,120);
	delay(20);
	moveForward(144,144);
	delay(20);
	/*
	for(int pwm=180; pwm<(speedLimit+1); pwm++){
		analogWrite(Motor_Left_Enable,pwm);
		analogWrite(Motor_Right_Enable,pwm);
		delayMicroseconds((startTime-20)*1000/256);
		//Serial.println(pwm);
	}*/
	//Serial.println("Smooth Start");
}

void rotateAntiClockwise90(int speedLimit=200, int startTime=200) {

	int beginEncoderCount = getRighttEncoderCount();

	rotateAntiClockwise(255,255);

	for(int pwm=150; pwm<(speedLimit+1); pwm++){
		rotateAntiClockwise(pwm,pwm);
		delayMicroseconds(startTime*1000/256);
	}

	while((getRighttEncoderCount()-beginEncoderCount)<15)
		rotateAntiClockwise(speedLimit,speedLimit);

	while((getRighttEncoderCount()-beginEncoderCount)<23)
		rotateAntiClockwise(150, 150);

	//smoothStop();
	//stop_soft();
	stop();
	delay(20);
	//test
	Serial.print(" Encoder count for -90:");
	Serial.println(getRighttEncoderCount()-beginEncoderCount);
}

void rotateClockwise90(int speedLimit=200, int startTime=200){

	int beginEncoderCount = getRighttEncoderCount();

	rotateClockwise(255,255);

	for(int pwm=150; pwm<(speedLimit+1); pwm++){
		rotateClockwise(pwm,pwm);
		delayMicroseconds(startTime*1000/256);
	}

	while((getRighttEncoderCount()-beginEncoderCount)<15)
		rotateClockwise(speedLimit,speedLimit);

	while((getRighttEncoderCount()-beginEncoderCount)<20)
		rotateClockwise(150,150);
	
	//smoothStop();
	//stop_soft();
	stop();
	delay(20);
	//test
	Serial.print(" Encoder count for 90:");
	Serial.println(getRighttEncoderCount()-beginEncoderCount);
}

void rotateAntiClockwise90Smooth(int speedLimit=200, int startTime=200) {

	int beginEncoderCount = getRighttEncoderCount();
	//int encorderCount = getRighttEncoderCount()-beginEncoderCount;
	rotateClockwise(200,200);

	for(int pwm=150; pwm<(speedLimit+1); pwm++){
		rotateAntiClockwise(pwm,pwm);
		delayMicroseconds(startTime*1000/256);
	}

	while((getRighttEncoderCount()-beginEncoderCount)<18)
		rotateAntiClockwise(speedLimit,speedLimit);

	while((getRighttEncoderCount()-beginEncoderCount)<23)
		rotateAntiClockwise(150,150);
		
	stop();
	delay(20);
	//test
	Serial.print(" Encoder count for smooth -90:");
	Serial.println(getRighttEncoderCount()-beginEncoderCount);
}

void rotateClockwise90Smooth(int speedLimit=200, int startTime=200){

	int beginEncoderCount = getRighttEncoderCount();

	rotateClockwise(255,255);

	for(int pwm=150; pwm<(speedLimit+1); pwm++){
		rotateClockwise(pwm,pwm);
		delayMicroseconds(startTime*1000/256);
	}

	while((getRighttEncoderCount()-beginEncoderCount)<15)
		rotateClockwise(speedLimit,speedLimit);

	while((getRighttEncoderCount()-beginEncoderCount)<20)
		rotateClockwise(150,150);
	
	//smoothStop();
	//stop_soft();
	stop();
	delay(20);
	//test
	Serial.print(" Encoder count for smooth 90:");
	Serial.println(getRighttEncoderCount()-beginEncoderCount);
}


void rotateClockwise90bySensors(){
	/*if(SensorFrontLeft==WHITE || SensorFrontRight==WHITE){
		//rotateClockwise(150,150);
		while(SensorFrontLeft==WHITE || SensorFrontRight==WHITE)
			rotateClockwise(150,150);
	}*/

	int beginEncoderCount = getRighttEncoderCount();
	//rotateAntiClockwise(150,150);
	//while((getRighttEncoderCount()-beginEncoderCount)<10);
	while((getRighttEncoderCount()-beginEncoderCount)<10){  
		rotateClockwise(170,230);
	}

	while((getRighttEncoderCount()-beginEncoderCount)<15){  
		rotateClockwise(130,180);
	}

	while(SensorFrontLeft==BLACK && SensorFrontRight==BLACK)
		rotateClockwise(110,140);

	/*
	while(SensorFrontLeft==WHITE && SensorFrontRight==WHITE)
		rotateClockwise(80,80);
		*/

	stop(10);
}

void rotateAntiClockwise90bySensors(){
	/*if(SensorFrontLeft==WHITE || SensorFrontRight==WHITE)
		while(SensorFrontLeft==WHITE || SensorFrontRight==WHITE)
			rotateAntiClockwise(150,150);
	*/
	/*
	int beginEncoderCount = getRighttEncoderCount();
	int EncorderCount = getRighttEncoderCount() - beginEncoderCount;

	rotateAntiClockwise(200,200);
	while(EncorderCount<10){  
		 EncorderCount = getRighttEncoderCount() - beginEncoderCount;//200,220
	}

	rotateAntiClockwise(150,150);
	while(EncorderCount<15){  
		EncorderCount = getRighttEncoderCount() - beginEncoderCount;
	}

	rotateAntiClockwise(120,120);
	EncorderCount = getRighttEncoderCount() - beginEncoderCount;
	while(SensorFrontLeft==BLACK && SensorFrontRight==BLACK && EncorderCount<25){
		EncorderCount = getRighttEncoderCount() - beginEncoderCount;
	}
	*/
	
	int beginEncoderCount = getRighttEncoderCount();
	//rotateAntiClockwise(150,150);
	//while((getRighttEncoderCount()-beginEncoderCount)<10);
	while((getRighttEncoderCount()-beginEncoderCount)<10){  
		rotateAntiClockwise(230,170);
	}

	while((getRighttEncoderCount()-beginEncoderCount)<15){  
		rotateAntiClockwise(180,130);
	}

	while(SensorFrontLeft==BLACK && SensorFrontRight==BLACK)
		rotateAntiClockwise(140,110);
		

	/*
	while(SensorFrontLeft==WHITE && SensorFrontRight==WHITE)
		rotateAntiClockwise(20,20);*/

	stop(100);

	//Serial.println();
	//Serial.print("Anti Clockwise: ");
	//Serial.println(getRighttEncoderCount()-beginEncoderCount);
}

void rotateClockwise180bySensors(){

	
	if(SensorFrontLeft==WHITE || SensorFrontRight==WHITE)
		while(SensorFrontLeft==WHITE || SensorFrontRight==WHITE)
			rotateClockwise(200,200);

	int beginEncoderCount = getRighttEncoderCount();
	while((getRighttEncoderCount()-beginEncoderCount)<15){  
		rotateClockwise(255,255);
	}

	while(SensorFrontLeft==BLACK && SensorFrontRight==BLACK)
		rotateClockwise(255,255);

	while(SensorFrontLeft==WHITE || SensorFrontRight==WHITE)
		rotateClockwise(255,255);

	beginEncoderCount = getRighttEncoderCount();
	while((getRighttEncoderCount()-beginEncoderCount)<2){  
		rotateClockwise(150,150);
	}

	rotateClockwise(0,0);
	delay(20);

	while(SensorFrontLeft==BLACK && SensorFrontRight==BLACK)
		rotateClockwise(100,100);
	
	stop(50);

	/*
	beginEncoderCount = getRighttEncoderCount();
	while((getRighttEncoderCount()-beginEncoderCount)<4){  
		rotateAntiClockwise(150,150);
	}

	stop(50);
	*/
}

void rotateAntiClockwise180bySensors(){
	
	if(SensorFrontLeft==WHITE || SensorFrontRight==WHITE)
		while(SensorFrontLeft==WHITE || SensorFrontRight==WHITE)
			rotateAntiClockwise(200,200);

	int beginEncoderCount = getRighttEncoderCount();
	while((getRighttEncoderCount()-beginEncoderCount)<15){  
		rotateAntiClockwise(255,255);
	}

	while(SensorFrontLeft==BLACK && SensorFrontRight==BLACK)
		rotateAntiClockwise(255,255);

	while(SensorFrontLeft==WHITE || SensorFrontRight==WHITE)
		rotateAntiClockwise(255,255);

	beginEncoderCount = getRighttEncoderCount();
	while((getRighttEncoderCount()-beginEncoderCount)<2){  
		rotateAntiClockwise(150,150);
	}

	rotateClockwise(0,0);
	delay(20);

	while(SensorFrontLeft==BLACK && SensorFrontRight==BLACK)
		rotateAntiClockwise(100,100);
	
	stop(50);

	/*
	beginEncoderCount = getRighttEncoderCount();
	while((getRighttEncoderCount()-beginEncoderCount)<4){  
		rotateClockwise(150,150);
	}

	stop(50);*/
}

void rotateClockwise180(){ 
	int beginEncoderCount = getRighttEncoderCount();

	rotateClockwise(255,255);

	for(int pwm=150; pwm<200; pwm++){
		rotateClockwise(pwm,pwm);
		delayMicroseconds(200*1000/256);
	}

	while((getRighttEncoderCount()-beginEncoderCount)<35)
		rotateClockwise(255,255);

	while((getRighttEncoderCount()-beginEncoderCount)<45)
		rotateClockwise(100,100);
	
	//smoothStop();
	//stop_soft();
	stop();
	delay(20);
	//test
	Serial.print(" Encoder count for 180:");
	Serial.println(getRighttEncoderCount()-beginEncoderCount);
}

void rotateAntiClockwise180(){ 
	int beginEncoderCount = getRighttEncoderCount();

	rotateAntiClockwise(255,255);

	for(int pwm=150; pwm<200; pwm++){
		rotateAntiClockwise(pwm,pwm);
		delayMicroseconds(200*1000/256);
	}

	while((getRighttEncoderCount()-beginEncoderCount)<40)
		rotateAntiClockwise(255,255);

	while((getRighttEncoderCount()-beginEncoderCount)<50)
		rotateAntiClockwise(150, 150);

	//smoothStop();
	//stop_soft();
	stop();
	delay(20);
	//test
	//Serial.print(" Encoder count for -90:");
	//Serial.println(getRighttEncoderCount()-beginEncoderCount);
}

boolean gotoAdjacentNode(node nextNode){

	int direction = isAdjacent(currentPosition, nextNode);

	if(turnToDirection(direction)){
		updateState(direction,nextNode);
		gotoNextFrontNode();
		return true;
	}
	return false;
}

boolean gotoAdjacentNodeSlow(node nextNode){

	int direction = isAdjacent(currentPosition, nextNode);

	if(turnToDirection(direction)){
		updateState(direction,nextNode);
		gotoNextFrontNodeSlow();
		return true;
	}
	return false;
}

boolean gotoAdjacentNodeSlowSlow(node nextNode){

	int direction = isAdjacent(currentPosition, nextNode);

	if(turnToDirection(direction)){
		updateState(direction,nextNode);
		gotoNextFrontNodeUnknownDistance();
		return true;
	}
	return false;
}

boolean gotoAdjacentNodeDistanceUnknown(node nextNode){

	int direction = isAdjacent(currentPosition, nextNode);

	if(turnToDirection(direction)){
		updateState(direction,nextNode);
		gotoNextFrontNodeUnknownDistance();
		return true;
	}
	return false;
}

void gotoNextFrontNode(){

	totalError = 0;

	int beginEncoderCount;

	smoothStart();
	beginEncoderCount = getRighttEncoderCount();

	if(isNode()){
		//while((getRighttEncoderCount()-beginEncoderCount)<4)
		moveForward(150,150);
	}

	while((getRighttEncoderCount()-beginEncoderCount)<4)
		line_following_pd_slow();

	while((getRighttEncoderCount()-beginEncoderCount)<35){
		line_following_pd();
	}

	while((SensorMiddleLeft == BLACK || SensorMiddleRight==BLACK) && Scout==WHITE){
		line_following_pd_slow();
	}

	while(!isNode()){
		line_following_pd_slow_slow();
	}
	/*
	stop(1);
	
	if(!isNode())
		while(!isNode()){
			moveForward(120,120);
		}
	*/
	stop(5);
	//now robot at on a node
}

void gotoNextFrontNodeUnknownDistance(){

	totalError = 0;

	int beginEncoderCount;

	smoothStart();
	if(isNode()){

		beginEncoderCount = getRighttEncoderCount();

		while((getRighttEncoderCount()-beginEncoderCount)<4)
			moveForward(120,120);
	}
	while(!isNode()){  //
		line_following_pd_slow_slow();
	}
	/*
	while(!isNode()){

		moveForward(80,80);
	}
	*/
	stop();
	//now robot at on a node
}

void gotoNextFrontNodeSlow(){

	totalError = 0;

	int beginEncoderCount;


	smoothStart();
	beginEncoderCount = getRighttEncoderCount();


	if(isNode()){
		//Serial.println("it is yet node");
		while(isNode() && (getRighttEncoderCount()-beginEncoderCount)<4)
		moveForward(150,150);
	}

	//Serial.println("node is passed");
	while((getRighttEncoderCount()-beginEncoderCount)<4)
		line_following_pd_slow();

	//Serial.println("accelerating");
	while((getRighttEncoderCount()-beginEncoderCount)<40){
		line_following_pd_slow();
	}

	//Serial.println("there will be a junction: slowing down");
	while((SensorMiddleLeft == BLACK || SensorMiddleRight==BLACK) && Scout==WHITE){
		line_following_pd_slow();
	}

	//Serial.println("junction detected in front: slow motion");
	while(!isNode()){
		line_following_pd_slow_slow();
	}
	/*
	stop(1);
	
	if(!isNode())
		while(!isNode()){
			moveForward(120,120);
		}
	*/

	
	stop(5);
	//Serial.println("Now at the junction");
	//now robot at on a node
}


boolean turnToDirection(int direction){

	if(direction == -1)
		return false;
	
	switch(turningAngles[currentDirection][direction]){
		case  0: 
			//Serial.write("0 ");
			break;
		case  1: 
			//Serial.write("90 ");
			rotateClockwise90bySensors(); 
			break;
		case -1: 
			//Serial.write("-90 ");
			rotateAntiClockwise90bySensors(); 
			break;
		case  2: 
			//Serial.write("180 ");
			rotateAntiClockwise180bySensors(); 
			break;
		default: 
			break;
	}
	currentDirection = direction;
	return true;
}

/*!
 * @function    turnTowardAdjacentNode
 * @abstract    turn toward a adjacent node.
 * @discussion  turn the autonomous bot toward given adjacent node,
 *              if they are adjacent.
 * @param       currentNode    The current node that autonomous bot is standing.
 * @param       adjacentNode   The adjacent node that autonomous bot should turn towards.
 * @result      If they are not adjacent returns -1. if they are adjacent returns the direction of node2 with respect to node1
 */
void turnTowardAdjacentNode(node currentNode, node adjacentNode){

	int newDirection = isAdjacent(currentNode,adjacentNode);

	if( newDirection != -1 ){
		turnToDirection(newDirection);
	}
	else{
		return;
	}
}

/*!
 * @function    isAdjacent
 * @abstract    check that if two nodes are adjacent
 * @discussion  This function take a two nodes.
 *              Check whether they are adjacent.
 * @param       node1    first node of node couple. reference direction will be with respect to this node.
 * @param       node2    second node of node couple
 * @result      If they are not adjacent returns -1. if they are adjacent returns the direction of node2 with respect to node1
 */
int isAdjacent(node node1, node node2){
	if(node1.x == node2.x){
		if(node1.y-node2.y==1){
			return WEST;
		}
		else if(node1.y-node2.y==-1){
			return EAST;
		}
		else{
			return -1;
		}
	}else if(node1.y == node2.y){
		if(node1.x-node2.x==1){
			return SOUTH;
		}
		else if(node1.x-node2.x==-1){
			return NORTH;
		}
		else{
			return -1;
		}
	}else{
		return -1;
	}
}

//abandoned
void setPathfolowingPID(boolean isWithBox){
	if(isWithBox)
		setPID(0.16,0.001,6);  // change this values; should be larger...
	else
		setPID(0.16,0.001,6); 
}

#endif