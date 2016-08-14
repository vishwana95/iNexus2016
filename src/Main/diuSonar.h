// Authors: Diunuge Buddhika Wijesinghe
// Competition iNexus 2013
// version: 2.0v: Not Optimized
// Last mod: 12-11-2012


#ifndef diuSonar_H
#define diuSonar_H

#include <NewPing.h>

typedef NewPing Sonar;

/*! @define TRIGGER_PIN_FRONT  Trigger pin of the front ultrasonic sensor */
#define TRIGGER_PIN_FRONT 12
/*! @define TRIGGER_PIN_FRONT  Echo pin of the front ultrasonic sensor */
#define ECHO_PIN_FRONT    13 

/*! @define TRIGGER_PIN_RIGHT  Trigger pin of the right-side ultrasonic sensor */
#define TRIGGER_PIN_RIGHT 44
/*! @define ECHO_PIN_RIGHT  Echo pin of the right-side ultrasonic sensor */
#define ECHO_PIN_RIGHT    45   

/*! @define TRIGGER_PIN_LEFT  Trigger pin of the left-side ultrasonic sensor */
#define TRIGGER_PIN_LEFT  41  
/*! @define ECHO_PIN_LEFT  Echo pin of the left-side ultrasonic sensor */
#define ECHO_PIN_LEFT     40

/*! @define MAX_DISTANCE  Maximum distance we want to check for (in centimeters). Maximum sensor distance is rated at 400-500cm. But in this application it is about 180cm */
#define MAX_DISTANCE 200

/*!
 * @var         sonarFront
 * @abstract    Sonar object which handle front sonar sensor.
 * @discussion  We can get object distance using this object in cm. Also it take-cares about noise handling.
 */
Sonar sonarFront(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
Sonar sonarRight(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
Sonar sonarLeft(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


void initializeSonar() 
{ 
	
} 

void testSonar(){
	delay(50);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
	unsigned int uS = sonarLeft.ping(); // Send ping, get ping time in microseconds (uS).
	Serial.print("Left: ");
	Serial.print(uS / US_ROUNDTRIP_CM); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
	Serial.print("cm\t");

	delay(50);  
	uS = sonarFront.ping(); // Send ping, get ping time in microseconds (uS).
	Serial.print("Front: ");
	Serial.print(uS / US_ROUNDTRIP_CM); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
	Serial.print("cm\t");

	delay(50);  
	uS = sonarRight.ping(); // Send ping, get ping time in microseconds (uS).
	Serial.print("Right: ");
	Serial.print(uS / US_ROUNDTRIP_CM); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
	Serial.println("cm");

	delay(200);  
}


void testSonarLeft(){
	delay(50);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
	unsigned int uS = sonarLeft.ping(); // Send ping, get ping time in microseconds (uS).
	Serial.print("Ping: ");
	Serial.print(uS / US_ROUNDTRIP_CM); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
	Serial.println("cm");
}

unsigned int getFrontSonarReading(){

	return (unsigned int)(sonarFront.ping()/US_ROUNDTRIP_CM); 
}

unsigned int getRightSonarReading(){
	return sonarRight.ping()/US_ROUNDTRIP_CM; 
}

unsigned int getLeftSonarReading(){
	return sonarLeft.ping()/US_ROUNDTRIP_CM; 
}

/*
unsigned short getDistanceSonar2(char Sensor){
       unsigned double x,y;
       unsigned int L;
       unsigned int H;
       TMR1H=0x00;
       TMR1L=0x00;
       if(Sensor=='M'){

                  TRIG_MIDDLE = 0;
                  delay_us(2);
                  TRIG_MIDDLE = 1;
                  delay_us(5);
                  TRIG_MIDDLE =0;

                  while(!ECHO_MIDDLE)
                  {
                      L=(TMR1L);
                      H=(TMR1H);
                      if(H>0xF0)
                               break;
                  }
                  TMR1H=0;
                  TMR1L=0;
                  T1CON.TMR1ON=1;
                  while(ECHO_MIDDLE)
                  {
                      L=(TMR1L);
                      H=(TMR1H);
                      if(H>0xF0)
                                    break;
                  }
                  L=(TMR1L);
                  H=(TMR1H);
                  x=(H*256 + L)*0.2;
                  y= (x/2)/29.1;
                  return (int)y;
       }
       else if(Sensor == 'R'){
                  TRIG_RIGHT = 0;
                  delay_us(2);
                  TRIG_RIGHT = 1;
                  delay_us(5);
                  TRIG_RIGHT =0;

                  while(!ECHO_RIGHT)
                  {
                      L=(TMR1L);
                      H=(TMR1H);
                      if(H>0xF0)
                                break;
                  }
                  TMR1H=0;
                  TMR1L=0;
                  T1CON.TMR1ON=1;
                  while(ECHO_RIGHT)
                  {
                      L=(TMR1L);
                      H=(TMR1H);
                      if(H>0xF0)
                                    break;
                  }
                  L=(TMR1L);
                  H=(TMR1H);
                  x=(H*256 + L)*0.2;
                  y= (x/2)/29.1;
                  return (int)y;
       }
       else if(Sensor == 'L'){
                  TRIG_LEFT = 0;
                  delay_us(2);
                  TRIG_LEFT = 1;
                  delay_us(5);
                  TRIG_LEFT =0;

                  while(!ECHO_LEFT)
                  {
                      L=(TMR1L);
                      H=(TMR1H);
                      if(H>0xF0)
                                break;
                  }
                  TMR1H=0;
                  TMR1L=0;
                  T1CON.TMR1ON=1;
                  while(ECHO_LEFT)
                  {
                      L=(TMR1L);
                      H=(TMR1H);
                      if(H>0xF0)
                                    break;
                  }
                  L=(TMR1L);
                  H=(TMR1H);
                  x=(H*256 + L)*0.2;
                  y= (x/2)/29.1;
                  return (int)y;
       }
}
*/

#endif