// Authors: Diunuge Buddhika Wijesinghe
// Competition iNexus 2013
// version: 2.0v: Not Optimized
// Last mod: 11-11-2012


#ifndef diuTest_H
#define diuTest_H

#include "motion.h"
#include "sensorPanel.h"
#include "dataTypes.h"
#include "algorithm.h"
#include "diuServo.h"
#include "sharpIR.h"
#include "diuSonar.h"
#include "indicator.h"


/************************** function prototypes ************************/

void testSerial();
void testImmediateStop();
void testGoTroughTGrid();
void testDirectionChange();


/************************** function definitions ************************/

void testSerial(){
	for(int counter=0; counter<5; counter++){
		Serial.write("Test");
		delay(1000);
	}
}

void testImmediateStop(){

	Serial.write("Foward");
	moveForward(200,200);
	delay(2000);
	Serial.write("Stop");
	stop();
	delay(1000);

	Serial.write("Foward");
	moveForward(200,200);
	delay(2000);
	Serial.write("Stop");
	stop();
	delay(1000);

	Serial.write("Foward");
	moveForward(200,200);
	delay(2000);
	Serial.write("Stop");
	stop();
	delay(1000);

	Serial.write("Foward");
	moveForward(200,200);
	delay(2000);
	Serial.write("Stop");
	stop();
	delay(1000);
}

void testSmoothRun(){
	//int testTime = 200;
	Serial.write("Foward");
	//moveForward(200,200);
	smoothStart();
	delay(200);
	Serial.write("Stop");
	smoothStop();
	delay(200);
}

void testGoTroughTGrid(){
	Serial.write("First node  ");
	gotoNextFrontNode();
	stop();
	Serial.write("node reached  ");
	delay(500);
	gotoNextFrontNode();
	stop();
	Serial.write("node reached  ");
	delay(500);
	gotoNextFrontNode();
	stop();
	Serial.write("node reached  ");
	delay(500);
	gotoNextFrontNode();
	stop();
	Serial.write("node reached  ");
	delay(500);
	rotateClockwise90();
	gotoNextFrontNode();
	stop();
	Serial.write("node reached  ");
	delay(500);
	rotateClockwise90();
	gotoNextFrontNode();
	stop();
	Serial.write("node reached  ");
	delay(500);
	gotoNextFrontNode();
	stop();
	Serial.write("node reached  ");
	delay(500);
	gotoNextFrontNode();
	stop();
	Serial.write("node reached  ");
	delay(500);
}

void testDirectionChange(){
	turnToDirection(EAST);
	delay(1000);
	turnToDirection(NORTH);
	delay(1000);
	turnToDirection(WEST);
	delay(1000);
	turnToDirection(EAST);
	delay(1000);
	turnToDirection(SOUTH);
	delay(1000);
	turnToDirection(EAST);
	delay(5000);
}

void testGridPathCreate(){

	node nodes[16];
	path aPath;
	aPath.nodes = nodes;
	createDryRunPath(&aPath);
	showPath(&aPath);
}

void testgoThroughGrid(){
	node node1 = {1,1};
	node node2 = {1,2};
	node node3 = {1,3};
	node node4 = {1,4};
	node node5 = {2,4};
	node node6 = {2,3};
	node node7 = {2,2};
	node node8 = {2,1};
	gotoAdjacentNode(node1);
	delay(1000);
	gotoAdjacentNode(node2);
	delay(1000);
	gotoAdjacentNode(node3);
	delay(1000);
	gotoAdjacentNode(node4);
	delay(1000);
	gotoAdjacentNode(node5);
	delay(1000);
	gotoAdjacentNode(node6);
	delay(1000);
	gotoAdjacentNode(node7);
	delay(1000);
	gotoAdjacentNode(node8);
	delay(1000);

	delay(10000);
}

void testServoFrontGrip() 
{ 
	//initializeServo();
	releaseFrontBox();
	delay(3000);
	grabFrontBox();
	delay(1000);
	/*int pos;
	for(pos = 0; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees 
	{                                  // in steps of 1 degree 
		servoFrontGrip.write(pos);              // tell servo to go to position in variable 'pos' 
		delay(15);                       // waits 15ms for the servo to reach the position 
	} 
	for(pos = 180; pos>=1; pos-=1)     // goes from 180 degrees to 0 degrees 
	{                                
		servoFrontGrip.write(pos);              // tell servo to go to position in variable 'pos' 
		delay(15);                       // waits 15ms for the servo to reach the position 
	} */
}

void testRotation90(){
	rotateClockwise90();
	delay(1000);
	rotateAntiClockwise90();
	delay(1000);
}

void testIRSharpFront(){
	
	delay(100);
	Serial.println(readFrontSharpShortRange);
}

void testIRSharpDistance(){
	float volts = analogRead(FRONT_SHARP_SHORT_RANGE_PIN)*0.0048828125;   // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
	float distance = 27*pow(volts, -1.10);   // worked out from graph 65 = theretical distance / (1/Volts)S 
	Serial.print(distance);
}

void testSonarRun(){
	int theSensor=2;
	unsigned int distance;
	gotoNextFrontNode();
	for (int count=0; count<3; count++){
			
		if(isObjectDetected(&theSensor, &distance)){
			stop(2000);
			gotoNextFrontNode();
			//sendSonarReadings();
		}
		else
			gotoNextFrontNode();
	}
	delay(60000);
}

void testMotorSpeed(){

	Serial.println();
	Serial.println("Foward Test");
	for(int i=0; i<255; i++){
		moveForward(i,i);
		delay(100);
		Serial.println(i);
	}

	delay(3000);

	Serial.println();
	Serial.println();
	for(int i=255; i>=0; i--){
		moveForward(i,i);
		delay(100);
		Serial.println(i);
	}

	delay(5000);

	Serial.println();
	Serial.println();
	Serial.println("Foward Test");
	Serial.println();
	for(int i=0; i<255; i++){
		reverse(i,i);
		delay(100);
		Serial.println(i);
	}

	delay(3000);

	Serial.println();
	Serial.println();
	for(int i=255; i>=0; i--){
		reverse(i,i);
		delay(100);
		Serial.println(i);
	}

	delay(3000);
}


void testSmoothStart(){
	Serial.println();
	Serial.println("Smooth Start");
	smoothStart(200,255);

	delay(200);

	Serial.println("Stopping");
	stop();
	delay(3000);
}

void testSensorPanels(){

	Serial.println("Disabling Back Panel");
	delay(2000);
	backSensorPanel(DISABLE);
	digitalWrite(30,LOW);
	for(int i=0; i<10; i++){
		sendSensorStatusAnalog();
		delay(500);
	}
			
	Serial.println("Enabling Back Panel");
	delay(2000);
	backSensorPanel(ENABLE); 
	digitalWrite(30,HIGH);
	for(int i=0; i<50; i++){
		sendSensorStatusAnalog();
		delay(500);
	}
}

void tsetIndicators(){

	Serial.println("Blue....");
	blink(BLUE,5);
	delay(2000);
	Serial.println("Red....");
	blink(RED,5);
	delay(2000);
	Serial.println("Yellow....");
	blink(YELLOW,5);
	delay(2000);
}

void testSwitches(){
	Serial.println();
	if(getSwitchStatus(DRY_RUN_SELECTION_SWITCH) == ON)
		Serial.print("On    ");
	else
		Serial.print("Off   ");

	if(getSwitchStatus(SELECTION_SWITCH_0) == ON)
		Serial.print("On    ");
	else
		Serial.print("Off   ");

	if(getSwitchStatus(SELECTION_SWITCH_1) == ON)
		Serial.print("On    ");
	else
		Serial.print("Off   ");

	if(getSwitchStatus(SELECTION_SWITCH_2) == ON)
		Serial.print("On    ");
	else
		Serial.print("Off   ");
	Serial.println();
}

void testMotorDirections(){
	
	motorLeft_foward(200);
	motorRight_foward(200);
	delay(1000);
	motorRight_reverse(200);
	delay(3000);
}


void testGridRunWithServo(){
	path dryRunPath;
	node dryRunNodes[(NO_OF_GRID_ROWS-1)*(NO_OF_GRID_COLUMNS-1)];
	dryRunPath.nodes = dryRunNodes;
	createDryRunPath(&dryRunPath);
	
	for(int counter0=0; counter0<dryRunPath.length; counter0++){

		int direction = isAdjacent(currentPosition, dryRunPath.nodes[counter0]);

		if(turnToDirection(direction)){
			
			if(readFrontSharpShortRange<200 && readFrontSharpShortRange>100){
				//grab box
				while(readFrontSharpShortRange<600)
					line_following_pd();
				stop();
				delay(1000);
				grabFrontBox();
				delay(1000);
			}

			gotoNextFrontNode();
			updateState(direction,dryRunPath.nodes[counter0]);
			//gotoAdjacentNode(dryRunPath.nodes[counter0]);
		}

		
		delay(1000);
		releaseFrontBox();
		Serial.print(getRighttEncoderCount());
		Serial.print(" ");
	}
	DRY_RUN_STATE = RUN_TYPE_FINISHED;

}


#endif