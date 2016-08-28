// Authors: Diunuge Buddhika Wijesinghe
// Competition iNexus 2013
// version: 3.0v: Not Optimized
// Last mod: 29-12-2012
// Comment: updated for finals


#ifndef diuConfiguration_H
#define diuConfiguration_H


#include <wprogram.h>

/****************** coding helping definitions ********************/

/*! @define   Scout    Returns relevent digital state of Scout sensor from it's analog reading */
#define Scout   sensorDigitalState(Scout_PIN)
#define Sensor1 sensorDigitalState(Sensor1_PIN)
#define Sensor2 sensorDigitalState(Sensor2_PIN)
#define Sensor3 sensorDigitalState(Sensor3_PIN)
#define Sensor4 sensorDigitalState(Sensor4_PIN)
#define Sensor5 sensorDigitalState(Sensor5_PIN)
#define Sensor6 sensorDigitalState(Sensor6_PIN)
#define Sensor7 sensorDigitalState(Sensor7_PIN)
#define Sensor8 sensorDigitalState(Sensor8_PIN)
#define Sensor9 sensorDigitalState(Sensor9_PIN)

#define SensorBack1 sensorDigitalStateBack(SensorBack1_PIN)
#define SensorBack2 sensorDigitalStateBack(SensorBack2_PIN)
#define SensorBack3 sensorDigitalStateBack(SensorBack3_PIN)
#define SensorBack4 sensorDigitalStateBack(SensorBack4_PIN)
#define SensorBack5 sensorDigitalStateBack(SensorBack5_PIN)
#define SensorBack6 sensorDigitalStateBack(SensorBack6_PIN)

#define SensorFrontRight digitalRead(SensorFrontRight_PIN) //sensorDigitalStateDigital(SensorFrontRight_PIN)   //
#define SensorFrontLeft  digitalRead(SensorFrontLeft_PIN)//sensorDigitalStateDigital(SensorFrontLeft_PIN)     //

#define SensorMiddleRight digitalRead(SensorMiddleRight_PIN)//sensorDigitalStateDigital(SensorMiddleRight_PIN) //
#define SensorMiddleLeft  digitalRead(SensorMiddleLeft_PIN)  //sensorDigitalStateDigital(SensorMiddleLeft_PIN)   //

/******************  define variables status  ********************/

/*! @define   NO_OF_GRID_ROWS    Number of rows that grid has */
#define NO_OF_GRID_ROWS 7 //7 , tested 6
/*! @define   NO_OF_GRID_COLUMNS    Number of columns that grid has */
#define NO_OF_GRID_COLUMNS 6 //7 tested 6


/*! @define   BLACK    Sensor input state for the darker color of the path */
#define BLACK HIGH  //HIGH
/*! @define   BLACK    Sensor input state for the lighter color of the path */
#define WHITE LOW   //LOW

/*! @define   HIGH_THRESHOLD    Threshold value(0-1024) of input 'HIGH' for convert analog sensor pannel values to digital */
#define HIGH_THRESHOLD 500 // ~2.5v  //500
/*! @define   LOW_THRESHOLD     Threshold value(0-1024) of input 'LOW' for convert analog sensor pannel values to digital */
#define LOW_THRESHOLD  500 // ~1v   //500

/*! @define   HIGH_THRESHOLD    Threshold value(0-1024) of input 'HIGH' for convert the "back sensor pannel" analog values to digital */
#define HIGH_THRESHOLD_BACK 300 // ~2.5v  //500
/*! @define   LOW_THRESHOLD     Threshold value(0-1024) of input 'LOW' for convert the "back sensor pannel" analog values to digital */
#define LOW_THRESHOLD_BACK  300 // ~1v   //500

#define SLOW_PWM 180
#define MID_PWM  200
#define FAST_PWM 255 // changed 29 Dec from 200

/*! @define NORTH  Indicate the reference north which uses to navigate autonomus robot */
#define NORTH 0
#define EAST  1
#define SOUTH 2
#define WEST  3

#define ENABLE 1
#define DISABLE 0

#define WAITING_TIME_FOR_THE_BOXES 3000

/********************* globle variables ***********************/

/*!
 * @var         sensorPannel
 * @abstract    create a object of QTRSensorsAnalog.
 * @discussion  create a object of QTRSensorsAnalog to use analog pid.
 */
//QTRSensorsAnalog sensorPannel((unsigned char[]) {11, 10, 6, 3, 2, 1, 0, 4, 8}, 9);
QTRSensorsRC sensorPannel((unsigned char[]) {46, 47, 48, 49, 50, 51, 52, 53}, 8);

/*!
 * @var         sensors
 * @abstract    store analog values of sensor panel.
 * @discussion  Stores analog values(0-1024) of main sensor panel. this gets update from sensorPanel.readLine()
 */
//unsigned int sensors[9];
unsigned int sensors[8];

//unsigned int sensorValuesMax[9];
//unsigned int sensorValuesMin[9];
unsigned int sensorValuesMax[8];
unsigned int sensorValuesMin[8];

/*!
 * @var         sensorPannelBack
 * @abstract    create a object of QTRSensorsAnalog.
 * @discussion  create a object of QTRSensorsAnalog to use analog pid.
 */
QTRSensorsAnalog sensorPannelBack((unsigned char[]) {5, 12, 13, 14, 15, 7}, 6);

/*!
 * @var         sensorsBack
 * @abstract    store analog values of rear sensor panel.
 * @discussion  Stores analog values(0-1024) of main sensor panel. this gets update from sensorPanel.readLine()
 */
unsigned int sensorsBack[6];

unsigned int sensorValuesBackMax[6];
unsigned int sensorValuesBackMin[6];

int position,error,lastError,error_reverse,motorSpeed,position_reverse;


/*!
 * @var         turningAngles
 * @abstract    Stores the rotation angle to change direction.
 * @discussion  This gives rotation angles as encoded integer for change direstion.
 *              Used in "turnToDirection(int direction)" function.
 *              Angles  0=> 0;  1=> 90;  2=> 180;  -1=> 90 
 */
const int turningAngles[4][4] = {{ 0,-1, 2, 1}, 
								 { 1, 0,-1, 2}, 
								 { 2, 1, 0,-1}, 
								 {-1, 2, 1, 0}};



//parameters for PID algorithm
float Kp, Ki, Kd;
float Kp_slow, Ki_slow, Kd_slow;
float Kp_slow_slow, Ki_slow_slow, Kd_slow_slow;

float Kp_Back, Kd_Back ,Ki_Back;
float Kp_Back_slow, Kd_Back_slow ,Ki_Back_slow;
float Kp_Back_slow_slow, Kd_Back_slow_slow ,Ki_Back_slow_slow;

int deviation, previousDeviation;
float correction, totalError, integral_correction;

int MIN_RPM, MID_RPM, MAX_RPM, MAX_CORRECTION, TEST_RPM, PID_RightRPM, PID_LeftRPM;

unsigned int counter = 0;

#endif