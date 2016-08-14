// Authors: Diunuge Buddhika Wijesinghe
// Competition iNexus 2013
// version: 2.0v: Not Optimized
// Last mod: 13-11-2012


#ifndef diuSharp_H
#define diuSharp_H

/*! @define   FRONT_SHARP_SHORT_RANGE_PIN    Analog pin number which connected to front short range sharp distance sensor */
#define FRONT_SHARP_SHORT_RANGE_PIN   12
/*! @define   FRONT_SHARP_SHORT_RANGE_PIN    Analog pin number which connected to front short range sharp distance sensor */
#define FRONT_SHARP_LONG_RANGE_PIN    13
/*! @define   FRONT_SHARP_SHORT_RANGE_PIN    Analog pin number which connected to front short range sharp distance sensor */
#define RIGHT_SIDE_SHARP_PIN   14
/*! @define   FRONT_SHARP_SHORT_RANGE_PIN    Analog pin number which connected to front short range sharp distance sensor */
#define LEFT_SIDE_SHARP_PIN    15

#define readFrontSharpShortRange analogRead(FRONT_SHARP_SHORT_RANGE_PIN)
#define readFrontSharpLongRange analogRead(FRONT_SHARP_LONG_RANGE_PIN)
#define readRightSideSharp analogRead(RIGHT_SIDE_SHARP_PIN)
#define readLefSideSharp analogRead(LEFT_SIDE_SHARP_PIN)
//int getDistance

int getDistanceFrontSharpLongRange(){ //for 20-150
	float volts = readFrontSharpShortRange*0.0048828125;   // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3 
	float distance = 65*pow(volts, -1.10);   // worked out from graph 65 = theretical distance / (1/Volts)S 
	int distanceCM = (int)distance;
	Serial.println(distanceCM);
}

int getDistanceFrontSharpMidRange(){ //for 10-80
	float volts = readFrontSharpShortRange*0.0048828125;   // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3 
	float distance = 27*pow(volts, -1.10);   // worked out from graph 65 = theretical distance / (1/Volts)S 
	int distanceCM = (int)distance;
	Serial.println(distanceCM);
}

#endif