// Authors: Diunuge Buddhika Wijesinghe
// Competition iNexus 2013
// version: 2.0v: Not Optimized
// Last mod: 12-11-2012


#ifndef diuAlgorithm_H
#define diuAlgorithm_H


#define RightSonar -1
#define FrontSonar 0
#define LeftSonar 1

#include "configuration.h"
#include "motion.h"
#include "status.h"
#include "dataTypes.h"
#include "diuOptimalPath.h"
#include "indicator.h"
#include "diuEEPROM.h"
#include "encoder.h"

Dijkstra optimalPathFinder;

/************************** function prototypes ************************/

void createDryRunPath(path* dryRunPath);
void showPath(path* aPath);
node createNode(int x, int y);
void aStar();
void findPathDijkstra(node startingNode, node endNode, shortestPath* path);
void findPathCustom();
unsigned int getSecureRange(int position, int currentDirection, node *currentNode);


/************************** function definitions ************************/

/*!
 * @function    getDryRunPath
 * @abstract    Create dry run path
 * @discussion  This function take a path pointer variable which has assigned node array.
 *              this will create a node list and assign it to path->nodes and set path->length.
 * @param       dryRunPath    A pointer to a path struct variable which will be cunstruct in the function
 * @result      Construct the path variable. returns nothing.
*/
void createDryRunPath(path* dryRunPath){
	int counter1=0;
	int counter3=0;

	dryRunPath->length = (NO_OF_GRID_ROWS-1)*(NO_OF_GRID_COLUMNS-1);

	for(int counter0=0; counter0<(NO_OF_GRID_ROWS-1); counter0++){
		//dryRunPath->nodes[counter3] = createNode(1, 1);
		
		if(counter0%2==0){
			for(; counter1<(NO_OF_GRID_COLUMNS-1); counter1++){

				dryRunPath->nodes[counter3] = createNode(counter0, counter1);
				counter3++;
			}
			counter1 = NO_OF_GRID_COLUMNS-2;
		}
		else{
			for(; counter1>=0; counter1--){

				dryRunPath->nodes[counter3] = createNode(counter0, counter1);
				counter3++;
			}
			counter1 = 0;
		}
	}
}

void createDryRunPathAlternative(path* dryRunPath, int pathNumber){
	int counter1=0;
	int counter3=0;

	switch(pathNumber){

		case 0: 

			dryRunPath->length = (NO_OF_GRID_ROWS-1)*(NO_OF_GRID_COLUMNS-1) + 4;

			dryRunPath->nodes[counter3++] = createNode(0, 0);

			for(int counter0=1; counter0<(NO_OF_GRID_ROWS-1); counter0++){
			//dryRunPath->nodes[counter3] = createNode(1, 1);
		
				if(counter0%2==1){
					for(; counter1<(NO_OF_GRID_COLUMNS-1); counter1++){

						dryRunPath->nodes[counter3] = createNode(counter0, counter1);
						counter3++;
					}
					counter1 = NO_OF_GRID_COLUMNS-2;
				}
				else{
					for(; counter1>=0; counter1--){

						dryRunPath->nodes[counter3] = createNode(counter0, counter1);
						counter3++;
					}
					counter1 = 0;
				}
			}

			dryRunPath->nodes[counter3++] = createNode(4, 4);
			dryRunPath->nodes[counter3++] = createNode(3, 4);
			dryRunPath->nodes[counter3++] = createNode(2, 4);
			dryRunPath->nodes[counter3++] = createNode(1, 4);
			dryRunPath->nodes[counter3++] = createNode(0, 4);
			dryRunPath->nodes[counter3++] = createNode(0, 3);
			dryRunPath->nodes[counter3++] = createNode(0, 2);
			dryRunPath->nodes[counter3++] = createNode(0, 1);

			break;

		case 1:

			break;
	}
}

/*!
 * @function    getDryRunPath
 * @abstract    Create dry run path
 * @discussion  This function take a path pointer variable which indicate real path in the grid.
 *              this will transmit via RS232 to computer with apropriate format.
 * @param       aPath    A pointer to a path struct variable which will be transmit in the function
 * @result      Transmit the path variable. returns nothing.
*/
void showPath(path* aPath){
	Serial.println("");
	for(int counter0=0; counter0<aPath->length; counter0++){
		
		Serial.print(" (");
		Serial.print(aPath->nodes[counter0].x);
		Serial.print(",");
		Serial.print(aPath->nodes[counter0].y);
		Serial.print(") ");

		Serial.print("-> ");
	}
}

boolean isObjectDetected(int *theSensor, unsigned int *distance){

	delay(35);
	unsigned int right = sonarRight.ping()/US_ROUNDTRIP_CM;
	//Serial.print("Right reading: ");
	//Serial.print(right);
	delay(35);
	unsigned int front = sonarFront.ping()/US_ROUNDTRIP_CM;
	//Serial.print("  Front reading: ");
	//Serial.print(front);
	delay(35);
	unsigned int left = sonarLeft.ping()/US_ROUNDTRIP_CM;
	//Serial.print("  Left reading:  ");
	//Serial.print(left);
	//Serial.print("      ");
	//sendSonarReadings();

	unsigned int rightSecureRange = getSecureRange(RightSonar,currentDirection,&currentPosition);
	unsigned int frontSecureRange = getSecureRange(FrontSonar,currentDirection,&currentPosition);
	unsigned int leftSecureRange = getSecureRange(LeftSonar,currentDirection,&currentPosition);

	/*
	Serial.println();
	Serial.print("Direction: ");
	Serial.print(currentDirection);
	Serial.print("   Node: (");
	Serial.print(currentPosition.x);
	Serial.print(",");
	Serial.print(currentPosition.y);
	Serial.print(")  ");
	Serial.print("  Right Range: ");
	Serial.print(rightSecureRange);
	Serial.print("  Front Range: ");
	Serial.print(frontSecureRange);
	Serial.print("  Left Range: ");
	Serial.println(leftSecureRange);
	*/

	//check the range
	int shift = (right+20)/30;
	if(right!=0 && right<rightSecureRange && shift!=0){
		//Serial.print("Object Detected at Right in:" );
		//Serial.println(right);
		//Serial.println();
		*theSensor = RightSonar;
		*distance = right;
		return true;
	}
	
	shift = (front+20)/30;
	if(front!=0 && front<frontSecureRange && shift!=0){
		//Serial.print("Object Detected at Front in:" );
		//Serial.println(front);
		//Serial.println();
		*theSensor = FrontSonar;
		*distance = front;
		return true;
	}
	
	shift = (left+20)/30;
	if(left!=0 && left<leftSecureRange && shift!=0){
		//Serial.print("Object Detected at Left in:" );
		//Serial.println(left);
		//Serial.println();
		*theSensor = LeftSonar;
		*distance = left;
		return true;
	}
	
	return false;
}


void createOptimalSearchPath(path *optimalSearchPath){
	optimalPathFinder.refreshGrid();
	//optimalPathFinder.printNodesStatus();
	boolean isPathFound=false;
	for(int row=NO_OF_GRID_ROWS-2; row>=0; row--){
		if(isPathFound)
			break;
		for(int column=0; column<NO_OF_GRID_COLUMNS-1; column++)
		{
			if(grid[row][column]==1)
				if(optimalPathFinder.getShortestPath(createNode(0,0), currentDirection ,createNode(row,column), optimalSearchPath)){
					isPathFound=true;
					break;
				}
		}
	}
}

unsigned int getSecureRange(int position, int currentDirection, node *currentNode){

	int sensorDirection= currentDirection+position;
	if(sensorDirection==4)
		sensorDirection=0;
	else if(sensorDirection==-1)
		sensorDirection=3;

	int distanceMultiplier=0;

	switch(sensorDirection){

	case 0:
		distanceMultiplier = NO_OF_GRID_ROWS-2-currentNode->x;
		if(distanceMultiplier==0)
			return 0;
		else if(currentPosition.y==0){
			return 30*1-8;
		}
		else
			return 30*distanceMultiplier-8;
	case 1: 
		distanceMultiplier = NO_OF_GRID_COLUMNS-2-currentNode->y;
		if(distanceMultiplier==0)
			return 0;
		else
			return 30*distanceMultiplier-8;
	case 2:
		distanceMultiplier = currentNode->x;
		if(distanceMultiplier==0)
			return 0;
		else if(currentPosition.y==0){
			return 30*1-8;
		}
		else
			return 30*distanceMultiplier-8;
	case 3:
		distanceMultiplier = currentNode->y;
		if(distanceMultiplier==0)
			return 0;
		else
			return 30*distanceMultiplier-8;
	}
}

/*!
 * @function    dryRunCalibration
 * @abstract    do the calibrations
 * @discussion  This function do the calibrations of main sensor panel and back sensor panel and save the calibration values.
 * @param       use arrays to save parameters
 * @result      write calibrated values to the EEPROM . returns nothing.
*/
void dryRunCalibration(){
	Serial.println("Starting the calibration.. ");
	blink(RED,2);

	for(counter=0; counter<300; counter++)
		sensorPannel.calibrate();
	
	blink(YELLOW,2);

	backSensorPanel(ENABLE);
	delay(50);
	for(counter=0; counter<300; counter++)
		sensorPannelBack.calibrate();
	
	backSensorPanel(DISABLE);

	blink(YELLOW,5);

	for(counter=0; counter<9; counter++){
		sensorValuesMax[counter] = sensorPannel.calibratedMaximumOn[counter];
		sensorValuesMin[counter] = sensorPannel.calibratedMinimumOn[counter];
	}
	eeprom_write_block((const void*)&sensorValuesMax, (void*)120, sizeof(sensorValuesMax));	
	eeprom_write_block((const void*)&sensorValuesMin, (void*)145, sizeof(sensorValuesMin));	

	blink(BLUE,1);

	for(counter=0; counter<6; counter++){
		sensorValuesBackMax[counter] = sensorPannelBack.calibratedMaximumOn[counter];
		sensorValuesBackMin[counter] = sensorPannelBack.calibratedMinimumOn[counter];
	}
	eeprom_write_block((const void*)&sensorValuesBackMax, (void*)165, sizeof(sensorValuesBackMax));	
	eeprom_write_block((const void*)&sensorValuesBackMin, (void*)185, sizeof(sensorValuesBackMin));	

	//for debug
	Serial.println("Calibrated values");

	Serial.println();

	Serial.print("Main Panel Maximum: ");
	for(counter=0; counter<9; counter++){
		Serial.print(sensorValuesMax[counter]);
		Serial.print("  ");
	}
	Serial.println();
	Serial.print("Main Panel Miniimum: ");
	for(counter=0; counter<9; counter++){
		Serial.print(sensorValuesMin[counter]);
		Serial.print("  ");
	}
	Serial.println();
	Serial.println();

	Serial.print("Back Panel Maximum: ");
	for(counter=0; counter<6; counter++){
		Serial.print(sensorValuesBackMax[counter]);
		Serial.print("  ");
	}
	Serial.println();
	Serial.print("Back Panel Miniimum: ");
	for(counter=0; counter<6; counter++){
		Serial.print(sensorValuesBackMin[counter]);
		Serial.print("  ");
	}
	Serial.println();

	for(counter=0; counter<2; counter++){
		blink(BLUE,1);
		blink(YELLOW,1);
		blink(RED,1);
	}
	Serial.println("End of the calibration.. ");
}

/*!
 * @function    loadCalibrations
 * @abstract    load the calibrations values
 * @discussion  This function loads the information aboout sensor panel calibration from previous calibrated values.
 * @param       use EEPROM
 * @result      read calibrated values from the EEPROM and update main sensor panel and back sensor panel controling objects. returns nothing.
*/
void loadCalibrations(){

	//Serial.println("Loading calibration...");
	eeprom_read_block((void*)&sensorValuesMax, (void*)120, sizeof(sensorValuesMax));
	eeprom_read_block((void*)&sensorValuesMin, (void*)145, sizeof(sensorValuesMin));
	eeprom_read_block((void*)&sensorValuesBackMax, (void*)165, sizeof(sensorValuesBackMax));
	eeprom_read_block((void*)&sensorValuesBackMin, (void*)185, sizeof(sensorValuesBackMin));

	//Serial.println("Read Complete...");
	//update the sensorPanel objects
	sensorPannel.calibrate();
	sensorPannelBack.calibrate();
	
	for(counter=0; counter<9; counter++){
		//Serial.print("i = ");
		//Serial.println(counter);
		sensorPannel.calibratedMaximumOn[counter] = sensorValuesMax[counter];
		sensorPannel.calibratedMinimumOn[counter] = sensorValuesMin[counter];
	}

	for(counter=0; counter<6; counter++){
		//Serial.print("i = ");
		//Serial.println(counter);
		sensorPannelBack.calibratedMaximumOn[counter] = sensorValuesBackMax[counter];
		sensorPannelBack.calibratedMinimumOn[counter] = sensorValuesBackMin[counter];
	}

	//for debug
	/*
	Serial.println("Calibrated values");

	Serial.println();

	Serial.print("Main Panel Maximum: ");
	for(counter=0; counter<9; counter++){
		Serial.print(sensorValuesMax[counter]);
		Serial.print("  ");
	}
	Serial.println();
	Serial.print("Main Panel Miniimum: ");
	for(int counter=0; counter<9; counter++){
		Serial.print(sensorValuesMin[counter]);
		Serial.print("  ");
	}
	Serial.println();
	Serial.println();

	Serial.print("Back Panel Maximum: ");
	for(counter=0; counter<6; counter++){
		Serial.print(sensorValuesBackMax[counter]);
		Serial.print("  ");
	}
	Serial.println();
	Serial.print("Back Panel Miniimum: ");
	for(counter=0; counter<6; counter++){
		Serial.print(sensorValuesBackMin[counter]);
		Serial.print("  ");
	}
	Serial.println();
	
	blink(BLUE,1);
	*/
}

void waitForTheLanding(){

	while(1){
		if(digitalRead(LANDING_DETECTION_PIN)==HIGH){
			delay(100);
			if(digitalRead(LANDING_DETECTION_PIN)==HIGH){
				delay(100);
				if(digitalRead(LANDING_DETECTION_PIN)==HIGH){
					break;
				}
			}
		}
	}

	//wait for adjesments
	delay(10); //3000
}

int getReverseDirection(int direction){
	direction +=2;
	if(direction==4)
		direction = 0;
	else if(direction==5)
		direction = 1;
	return direction;
}


#endif