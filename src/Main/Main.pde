// Authors: Diunuge Buddhika Wijesinghe
// Competition iNexus 2013
// version: 2.0v: Not Optimized
// Last mod: 12-11-2012

/********************* include libraries ***************************/

#include <Servo.h>
#include <LiquidCrystal.h>
#include <QTRSensors.h>
#include "dataTypes.h"
#include "motion.h"
#include "configuration.h"
#include "debug.h"
#include "test.h"
#include "status.h"
#include "diuEEPROM.h"
#include "encoder.h"
#include "diuServo.h"
#include "diuSonar.h"
#include "diuOptimalPath.h"
#include <NewPing.h>
#include "indicator.h"
#include "algorithm_beta.h"

/* setup code goes here */
void setup()
{
	//Do common initialization which is not dependable of type of run
	Serial.begin(9600);

	//configurations
	initializeSensorPanel();
	initializeMotors();
	initializeEncorders();
	initializeStatus();
	initializeServo();
	initializeIndicators();

	releaseFrontBox();
	
	//if this is dry run: find and save grid parameters: initialize variables
	if(isDryRun()){
		//initialize the grid parameters
		for(int counter0=0; counter0<(NO_OF_GRID_ROWS-1); counter0++)
			for(int counter1=0; counter1<(NO_OF_GRID_COLUMNS-1); counter1++){
				grid[counter0][counter1] = NODE_IS_ALLOWED;
			}
		
	}
	//if this is main run: load the parameters from dry run
	else{
		
		//Load grid status
		//eeprom_read_block((void*)&grid, (void*)0, sizeof(grid));

		//load transfer zone location
		//eeprom_read_block((void*)&transerzone, (void*)100, sizeof(transerzone));
	}

	for(int i=0; i<9; i++){
		sensorValuesMax[i] = 1023;
		sensorValuesMin[i] = 0;
	}

	for(int i=0; i<6; i++){
		sensorValuesBackMax[i] = 1023;
		sensorValuesBackMin[i] = 0;
	}

	
}

/****************    end of setup()   ********************************/

/* main loop after setup code */
void loop()
{
	//Serial.println();
	//Serial.println();
	Serial.write("Starting..");
	//delay(1000);

	setTest(true);
	setDryRun(false);
	if(isTesting()){
		//test codes goes here
		test();
	}

	setPID(8, 0.0, 16); 
	setPIDSlow(0.16, 0.0, 8.0);


	//Elavete the hand
	setHandElevation(180);

	//Section 1: Maze following

	while(!isMazeEnded()){

		if(getLeftSonarReading()>20){ //There is no left wall
			//Turn Left
			rotateAntiClockwise90();
		}
		else{
			if(getFrontSonarReading() > 10){ //There is no front wall
				//Move forward
				path_follow_PID_Sonar();
			}
			else{
				if(getRightSonarReading() > 20){ //There is no right wall
					//turn right
					rotateClockwise90();
				}
				else{
					//turn back
					rotateClockwise180();
				}
			}
		}
	}

	//Section 2: Grab Box
	//if there is space.. move forward
	moveForward(150,150);
	delay(100); //tune the delay
	stop();

	//Find the color

	setHandElevation(0);
	grabFrontBox();
	setHandElevation(180);

	// Section 3: Goto the correect path

	//Follow arrows
	while(!isPathEnded())
		path_follow_PID_Arrow();


	//Section 4: Drop the box
	//if there is space.. move forward
	moveForward(150,150);
	delay(100); //tune the delay
	stop();

	setHandElevation(0);
	releaseFrontBox();
	setHandElevation(180);

	/*
	if(isDryRun()){ //this is dry run

		DRY_RUN_STATE = RUN_TYPE_STARTED;
		Serial.println("Starting the dry run.. ");

		while(1){

			//switch dry run types
			
			if(DRY_RUN_STATE==RUN_TYPE_STARTED){

				//testSwitches();
				//calibrate
				if(getSwitchStatus(SELECTION_SWITCH_0)==OFF && getSwitchStatus(SELECTION_SWITCH_1)==OFF){
					dryRunCalibration();
				}

				//search nodes algo 1(using refresh grid nodes)
				else if(getSwitchStatus(SELECTION_SWITCH_0)==OFF && getSwitchStatus(SELECTION_SWITCH_1)==ON){
					Serial.println("Discover the grid.. ");
					//loadCalibrations();
					caliibrateSensorsBySensors();
					discoverGrid();
					sendGridStatus();  //debug

					findTransferZone();
				}

				//search nodes algo 2(using previous grid nodes)
				else if(getSwitchStatus(SELECTION_SWITCH_0)==ON && getSwitchStatus(SELECTION_SWITCH_1)==OFF){
					Serial.println("Discover the missing nodes.. ");
					loadCalibrations();
					discoverMissingNodes();
				}

				//search transfer zones
				else{
					Serial.println("Discover the transfer zones.. ");

					loadCalibrations();
					setPID(0.2,0.002,34);
					gotoNextFrontNodeUnknownDistance();
					gotoNextFrontNodeSlow();
					updateState(EAST, createNode(0,1));
					turnToDirection(NORTH);
					for(counter=0; counter<4; counter++)
						gotoNextFrontNode();
					gotoNextFrontNodeSlow();
					turnToDirection(WEST);
					gotoNextFrontNode();
					updateState(createNode(5,0));

					findTransferZone();
				}
				DRY_RUN_STATE = RUN_TYPE_FINISHED;
				Serial.println("End the dry run.. ");
			}

			else
				stop();
		}
	}


	else{
		Serial.println("Starting the competition... ");

		ACTIVE_RUN_STATE = RUN_TYPE_STARTED;

		eeprom_read_block((void*)&grid, (void*)0, sizeof(grid));
		eeprom_read_block((void*)&keyBlockTranserzone, (void*)100, sizeof(keyBlockTranserzone));
		eeprom_read_block((void*)&assemblyBlockTransferZone, (void*)110, sizeof(assemblyBlockTransferZone));

		//sendGridStatus();
		//printNode(&keyBlockTranserzone,"Key transfer zone node");
		//printNode(&assemblyBlockTransferZone,"Assembly transfer zone node");

		loadCalibrations();

		while(1){
			
			if(ACTIVE_RUN_STATE==RUN_TYPE_STARTED){

				if(getSwitchStatus(SELECTION_SWITCH_0)==OFF && getSwitchStatus(SELECTION_SWITCH_1)==OFF){
					waitForTheLanding();
					land();
					//printNode(&currentPosition, "Robot at");
				
					node intruderBox = findIntruderBox();
					//printNode(&intruderBox, "Intruder box at");
				
					depositeIntruderBox(&intruderBox);
				
					waitForTheKey();
					unlockTheSecureZone();

					waitForTheAssemblyBox();
					
				}
				else if(getSwitchStatus(SELECTION_SWITCH_0)==OFF && getSwitchStatus(SELECTION_SWITCH_1)==ON){
					land();
				
					node intruderBox = findIntruderBox();
					//printNode(&intruderBox, "Intruder box at");
				
					depositeIntruderBox(&intruderBox);
				
					waitForTheKey();
					unlockTheSecureZone();

					waitForTheAssemblyBox();
				}
				else if(getSwitchStatus(SELECTION_SWITCH_0)==ON && getSwitchStatus(SELECTION_SWITCH_1)==OFF){
					land();
					optimalPathFinder.refreshGrid();

					waitForTheKey();
					unlockTheSecureZone();

					waitForTheAssemblyBox();
				}
				else{
					land();
					optimalPathFinder.refreshGrid();

					waitForTheAssemblyBox();
				}

				saveTheEarth();
				
				finish();
				//for debug
				//sendGridStatus();
				ACTIVE_RUN_STATE = RUN_TYPE_FINISHED;
			}

			else
				stop();

		}
	}*/

	return;
}
/******************   end of loop()   ********************************/

int isMazeEnded(){

	return 0;
}

int isPathEnded(){

	return 0;
}

void test(){
	Serial.println("Testing.. ");

	setPID(8, 0.0, 16); 
	while(1){
			/*
			motorLeft_foward(150);
			delay(1000);
			motorLeft_stop();
			delay(2000);
			motorLeft_reverse(150);
			delay(1000);
			motorLeft_stop();
			delay(5000);

			motorRight_foward(150);
			delay(1000);
			motorRight_stop();
			delay(2000);
			motorRight_reverse(150);
			delay(1000);
			motorRight_stop();
			delay(5000);

			moveForward(150,150);
			delay(1000);
			stop();
			delay(2000);
			reverse(150,150);
			delay(1000);
			stop();
			delay(5000);
			*/

		//testSonar();

		
		if(false){
			if(getLeftSonarReading()>20){ //There is no left wall
				//Turn Left
				rotateAntiClockwise90();
			}
			else{
				if(getFrontSonarReading() > 10){ //There is no front wall
					//Move forward
					path_follow_PID_Sonar();
				}
				else{
					if(getRightSonarReading() > 20){ //There is no right wall
						//turn right
						rotateClockwise90();
					}
					else{
						//turn back
						rotateClockwise180();
					}
				}
			}
		}

		//Testing for RC sensor array
		if(false){

			for(int i=0; i<8; i++){
				sensorValuesMax[i] = 1023;
				sensorValuesMin[i] = 0;
			}

			// optional: wait for some input from the user, such as  a button press
 
			// then start calibration phase and move the sensors over both
			// reflectance extremes they will encounter in your application:
			Serial.println("Calibration is started!");

			int i;
			for (i = 0; i < 250; i++)  // make the calibration take about 5 seconds
			{
				sensorPannel.calibrate();
				delay(20);
			}


			Serial.println("Calibration is ended!");
			// optional: signal that the calibration phase is now over and wait for further
			// input from the user, such as a button press

			unsigned int position;

			for (i = 0; i < 250; i++)  // make the calibration take about 5 seconds
			{
				sensorPannel.read(sensors);

				Serial.print("1: ");
				Serial.print(sensors[0]);
				Serial.print("  2: ");
				Serial.print(sensors[1]);
				Serial.print("  3: ");
				Serial.print(sensors[2]);
				Serial.print("  4: ");
				Serial.print(sensors[3]);
				Serial.print("  5: ");
				Serial.print(sensors[4]);
				Serial.print("  6: ");
				Serial.print(sensors[5]);
				Serial.print("  7: ");
				Serial.print(sensors[6]);
				Serial.print("  8: ");
				Serial.print(sensors[7]);

				position = sensorPannel.readLine(sensors);

				Serial.print("\t  Potition: ");
				Serial.print(position);

				Serial.print("\t  Error: ");
				Serial.print(3500 - (int)position);

				Serial.println("");

				delay(1000);
			}
		}

		if(true){

			Serial.println("Calibration is started!");

			int i;
			for (i = 0; i < 250; i++)  // make the calibration take about 5 seconds
			{
				sensorPannel.calibrate();
				delay(20);
			}


			Serial.println("Calibration is ended!");

			setPIDSlow(0.16, 0.0, 8.0);

			while(true)
				path_follow_PID_Arrow();
		}
			
	}
}