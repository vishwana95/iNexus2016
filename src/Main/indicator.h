// Authors: Diunuge Buddhika Wijesinghe
// Competition iNexus 2013
// version: 3.0v: Not Optimized
// Last mod: 28-12-2012
// Comment: updated for finals


#ifndef indicator_H
#define indicator_H


#define YELLOW_PIN   19 
#define RED_PIN      18 
#define BLUE_PIN     20 
#define DISPLAY_PIN   4

#define YELLOW  0
#define RED     1
#define BLUE    2 
#define DISPLAY 3

void initializeIndicators(){
	
	pinMode(YELLOW_PIN,OUTPUT);
	pinMode(RED_PIN,OUTPUT);
	pinMode(BLUE_PIN,OUTPUT);
	pinMode(DISPLAY_PIN,OUTPUT);

	digitalWrite(YELLOW_PIN, LOW);
	digitalWrite(BLUE_PIN, LOW);
	digitalWrite(RED_PIN, LOW);
	digitalWrite(DISPLAY_PIN, LOW);
}

void blink(int indicator, int i){
	int indicator_PIN = RED_PIN;
	switch(indicator){

	case 0:
		indicator_PIN = YELLOW_PIN;
		break;
	case 1:
		indicator_PIN = RED_PIN;
		break;
	case 2:
		indicator_PIN = BLUE_PIN;
		break;
	case 3:
		indicator_PIN = DISPLAY_PIN;
		break;
	}

	for(int count=0; count<i; count++){
		digitalWrite(indicator_PIN, HIGH);
		delay(500);
		digitalWrite(indicator_PIN, LOW);
		delay(200);
	}
}

void display(int command){
	if( command = ENABLE )
		digitalWrite(DISPLAY, HIGH);
	else 
		digitalWrite(DISPLAY, LOW);
}

#endif