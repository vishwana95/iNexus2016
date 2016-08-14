// Authors: Diunuge Buddhika Wijesinghe
// Competition iNexus 2013
// version: 3.0v: Not Optimized
// Last mod: 29-12-2012
// Comment: updated for finals


#ifndef diuSensorPanel_H
#define diuSensorPanel_H

#include "configuration.h"

/*! @define   Scout_PIN    Analog pin number which connected to SCOUT(front) sensor */
#define Scout_PIN    9 
#define Sensor1_PIN 11
#define Sensor2_PIN 10
#define Sensor3_PIN  6 
#define Sensor4_PIN  3 
#define Sensor5_PIN  2 
#define Sensor6_PIN  1 
#define Sensor7_PIN  0 
#define Sensor8_PIN  4 
#define Sensor9_PIN  8 

#define SensorBack_Enable_PIN  30

#define SensorBack1_PIN  5
#define SensorBack2_PIN 12
#define SensorBack3_PIN 13
#define SensorBack4_PIN 14
#define SensorBack5_PIN 15
#define SensorBack6_PIN  7

#define SensorFrontRight_PIN 32
#define SensorFrontLeft_PIN  33

#define SensorMiddleRight_PIN 29
#define SensorMiddleLeft_PIN  28


boolean isBackSensorPanelEnabled;

/************************** function prototypes ************************/

void initializeSensorPanel();
int sensorDigitalState(int analogPIN);
int sensorDigitalStateDigital(int digitalPIN);
void backSensorPanel(int command=ENABLE);
boolean isNode();
boolean isBlockedNode();
boolean isAllowedNode();
boolean isAllWhite();
boolean isAllBlack();


/************************** function definitions ************************/

void initializeSensorPanel(){

	pinMode(Scout_PIN, INPUT);      
    pinMode(Sensor1_PIN, INPUT);      
	pinMode(Sensor2_PIN, INPUT);      
	pinMode(Sensor3_PIN, INPUT);      
	pinMode(Sensor4_PIN, INPUT);      
	pinMode(Sensor5_PIN, INPUT);      
	pinMode(Sensor6_PIN, INPUT);      
	pinMode(Sensor7_PIN, INPUT);      
	pinMode(Sensor8_PIN, INPUT);      
	pinMode(Sensor9_PIN, INPUT); 

	pinMode(SensorBack_Enable_PIN, OUTPUT); 
	backSensorPanel(DISABLE);

    pinMode(SensorBack1_PIN, INPUT);      
	pinMode(SensorBack2_PIN, INPUT);      
	pinMode(SensorBack3_PIN, INPUT);      
	pinMode(SensorBack4_PIN, INPUT);      
	pinMode(SensorBack5_PIN, INPUT);      
	pinMode(SensorBack6_PIN, INPUT); 

	pinMode(SensorFrontRight, INPUT);      
	pinMode(SensorFrontLeft, INPUT);

	pinMode(SensorMiddleRight_PIN, INPUT);      
	pinMode(SensorMiddleLeft_PIN, INPUT);
}

void backSensorPanel(int command){
	if(command==ENABLE){
		digitalWrite(SensorBack_Enable_PIN, HIGH);
		isBackSensorPanelEnabled = true;
	}
	else{
		digitalWrite(SensorBack_Enable_PIN, LOW);
		isBackSensorPanelEnabled = false;
	}
}


/*!
 * @function    sensorDigitalState
 * @abstract    Convert analog sensor state to digital
 * @discussion  This function take a analog pin number, read analog value,
 *              compare it with given threshold values, convert to relevant digital state.
 *              This can use for digital PID or normal line following
 * @param       analogPIN    The analog pin number. This must be an Integer. EX: 0 for A0
 * @param       HIGH_THRESHOLD  threshold value for digital high
 * @param       LOW_THRESHOLD   threshold value for digital low 
 * @result      A integer which indicate digital state.
 */
int sensorDigitalState(int analogPIN){
	int analogValue = analogRead(analogPIN);
	if(analogValue> HIGH_THRESHOLD)
		return BLACK;
	else
		return WHITE;
	/*
	//removed to get better performance

	else if(analogValue < LOW_THRESHOLD)
		return LOW;
	else
		return -1; // error status. this should be handled in future functions or remove this
	*/
}

int sensorDigitalStateBack(int analogPIN){
	int analogValue = analogRead(analogPIN);
	if(analogValue> HIGH_THRESHOLD_BACK)
		return BLACK;
	else
		return WHITE;
	/*
	else if(analogValue < LOW_THRESHOLD_BACK)
		return LOW;
	else
		return -1; // error status. this should be handled in future functions or remove this
	*/
}

//don't use in the actual grid
int sensorDigitalStateDigital(int digitalPIN){
	int digitalState = digitalRead(digitalPIN);
	if(digitalState == 1)
		return BLACK;
	else
		return WHITE;
}


boolean isNode(){
	if( Sensor1==WHITE && Sensor9==WHITE){ //Scout==WHITE && Sensor1==WHITE && Sensor2==WHITE && Sensor8==WHITE && Sensor9==WHITE
		return true;
    }
    else
        return false;
}

boolean isBlockedNode(){
	if( isNode() ){
		if((Sensor3==BLACK && Sensor4==BLACK) || (Sensor4==BLACK && Sensor5==BLACK) || (Sensor5==BLACK && Sensor6==BLACK) || (Sensor6==BLACK && Sensor7==BLACK))  //(Sensor4==BLACK && Sensor5==BLACK) || (Sensor5==BLACK && Sensor6==BLACK)
			return true;
		else
			return false;
    }
    else
        return false;
}

boolean isAllowedNode(){
	if( isNode() ){
		if((Sensor4==WHITE && Sensor5==WHITE) || (Sensor5==WHITE && Sensor6==WHITE))
			return true;
		else
			return false;
    }
    else
        return false;
}

boolean isAllBlack(){
      if( Scout==BLACK && Sensor1==BLACK && Sensor2==BLACK && Sensor3==BLACK && Sensor4==BLACK && Sensor5==BLACK && Sensor6==BLACK && Sensor7==BLACK && Sensor8==BLACK && Sensor9==BLACK){
            return true;
      }
      else
            return false;
}

boolean isAllWhite(){
      if( Scout==WHITE && Sensor1==WHITE && Sensor2==WHITE && Sensor3==WHITE && Sensor4==WHITE && Sensor5==WHITE && Sensor6==WHITE && Sensor7==WHITE && Sensor8==WHITE && Sensor9==WHITE){
            return true;
      }
      else
            return false;
}

#endif