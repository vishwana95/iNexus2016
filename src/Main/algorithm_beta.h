#ifndef diuAlgorithm_beta_H
#define diuAlgorithm_beta_H

#include "motion.h"
#include "sensorPanel.h"
#include "dataTypes.h"
#include "algorithm.h"
#include "diuServo.h"
#include "sharpIR.h"
#include "diuSonar.h"
#include "indicator.h"

void gotoReverseNodeUnknownDistance();

/*!
 * @function    discoverGrid
 * @abstract    Discover Grid while the dry run
 * @discussion  When this function called. Autonomous bot will generate the dry run path.
 *              Go through it. And record data about nodes(junctions), obstacles and landing zones.
 *              After that it'll write that data to EEPROM.
 * @param       NO_OF_GRID_ROWS      Number of rows that grid has; for calculation
 * @param       NO_OF_GRID_COLUMNS   Number of columns that grid has(must use sign convention) 
 * @result      EEPROM will have grid data. returns nothing.
 */
void discoverGrid(){
	path dryRunPath;
	node dryRunNodes[(NO_OF_GRID_ROWS-1)*(NO_OF_GRID_COLUMNS-1)];
	dryRunPath.nodes = dryRunNodes;
	createDryRunPath(&dryRunPath);
	
	//debug
	showPath(&dryRunPath);

	gotoAdjacentNodeDistanceUnknown(dryRunPath.nodes[0]);
	updateState(dryRunPath.nodes[0]);
	
	for(int counter0=1; counter0<dryRunPath.length; counter0++){

		if(counter0 == (dryRunPath.length-1))
			gotoAdjacentNodeDistanceUnknown(dryRunPath.nodes[counter0]);
		else if(isAdjacent(dryRunPath.nodes[counter0], dryRunPath.nodes[counter0+1]) != currentDirection)
			gotoAdjacentNodeDistanceUnknown(dryRunPath.nodes[counter0]);
		else
			gotoAdjacentNodeDistanceUnknown(dryRunPath.nodes[counter0]);

		stop(50);
		printNode(&dryRunPath.nodes[counter0],"Now at");
		sendSensorStatus();
		//delay(1000);
		//printNode(&currentPosition,"Current Node");
		if(isBlockedNode()){
			grid[dryRunPath.nodes[counter0].x][dryRunPath.nodes[counter0].y] = NODE_IS_BLOCKED;
			blink(RED,1);
		}
		else{
			int beginEncoderCount = getRighttEncoderCount();
			while(getRighttEncoderCount()-beginEncoderCount <3)
				moveForward(100,100);
			stop();
			sendSensorStatus();
			//delay(1000);
			if(isBlockedNode()){
				grid[dryRunPath.nodes[counter0].x][dryRunPath.nodes[counter0].y] = NODE_IS_BLOCKED;
				blink(RED,1);
			}
		}
		//delay(50);
		//Serial.print(getRighttEncoderCount());
		//Serial.print(" ");
	}

	//EEPROM_writeData(0,grid);
	eeprom_write_block((const void*)&grid, (void*)0, sizeof(grid));
	//indicate about finish
}

void discoverMissingNodes(){
	path dryRunPath;
	node dryRunNodes[(NO_OF_GRID_ROWS-1)*(NO_OF_GRID_COLUMNS-1)+10]; // 10 -> 4
	dryRunPath.nodes = dryRunNodes;
	createDryRunPathAlternative(&dryRunPath,0); // create advanced dry run path 
	
	//debug
	showPath(&dryRunPath);

	eeprom_read_block((void*)&grid, (void*)0, sizeof(grid));
	Serial.println("previous grid");
	sendGridStatus();

	gotoAdjacentNodeDistanceUnknown(dryRunPath.nodes[0]);
	updateState(dryRunPath.nodes[0]);
	
	for(int counter0=1; counter0<dryRunPath.length; counter0++){

		if(counter0 == (dryRunPath.length-1))
			gotoAdjacentNodeSlow(dryRunPath.nodes[counter0]);
		else if(isAdjacent(dryRunPath.nodes[counter0], dryRunPath.nodes[counter0+1]) != currentDirection)
			gotoAdjacentNodeSlow(dryRunPath.nodes[counter0]);
		else
			gotoAdjacentNode(dryRunPath.nodes[counter0]);

		//printNode(&currentPosition,"Current Node");
		if(isBlockedNode() && grid[dryRunPath.nodes[counter0].x][dryRunPath.nodes[counter0].y]==0){
			grid[dryRunPath.nodes[counter0].x][dryRunPath.nodes[counter0].y] = NODE_IS_BLOCKED;
			blink(RED,1);
		}

		delay(50);
	}

	eeprom_write_block((const void*)&grid, (void*)0, sizeof(grid));
	//indicate about finish
}

int findTransferZone(){

	printNode(&currentPosition, "Robot at");
	
	setReversePID(0.05,0.0001,16);
	setPID(0.2,0.002,34);

	/*
	if(currentPosition.x!=5 || currentPosition.y!=0){
		path pathToTransferZone;
		node transferZonePathNodes[(NO_OF_GRID_ROWS-1)*(NO_OF_GRID_COLUMNS-1)];
		pathToTransferZone.nodes = transferZonePathNodes;
		
		optimalPathFinder.getShortestPath(currentPosition, currentDirection, createNode(5,0), &pathToTransferZone);

		Serial.println("Path to transfer zone");
		showPath(&pathToTransferZone);
	
		for(int counter0=0; counter0<(pathToTransferZone.length-1); counter0++){

			gotoAdjacentNode(pathToTransferZone.nodes[counter0]);
			delay(50);
		}
		gotoAdjacentNodeSlow(pathToTransferZone.nodes[pathToTransferZone.length-1]);
		delay(50);

		printNode(&currentPosition, "Now, Robot at");
	}*/

	
	while(!isKeyBlockTransferZoneFound || !isAssemblyBlockTransferZoneFound){
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		delay(500);
		turnToDirection(WEST);

		//first place
		Serial.println("Go through first zone");

		backSensorPanel(ENABLE);

		if(isNode())
			while(isNode())
				moveForward(150,150);

		delay(10);

		int beginEncoderCount = getRighttEncoderCount();
		int EnCounter = getRighttEncoderCount()-beginEncoderCount;

		while(EnCounter<36){
			line_following_pd_slow_slow();

			EnCounter = getRighttEncoderCount()-beginEncoderCount;
			if(EnCounter>14 && (Sensor3==WHITE || Sensor7 == WHITE)){

				stop();
				blink(YELLOW, 2);

				if(isKeyBlockTransferZoneFound || isAssemblyBlockTransferZoneFound){ //one transfer zone is already found

					if(isKeyBlockTransferZoneFound && keyBlockTranserzone.x != currentPosition.x){
						assemblyBlockTransferZone.x = currentPosition.x;

						//write transfer zone location to EEPROM
						eeprom_write_block((const void*)&assemblyBlockTransferZone, (void*)110, sizeof(assemblyBlockTransferZone));	
						isAssemblyBlockTransferZoneFound = true;

						printNode(&assemblyBlockTransferZone, "assemblyBlockTransferZone");
					}
					else if(isAssemblyBlockTransferZoneFound && assemblyBlockTransferZone.x != currentPosition.x){
						keyBlockTranserzone.x = currentPosition.x;

						//write transfer zone location to EEPROM
						eeprom_write_block((const void*)&keyBlockTranserzone, (void*)100, sizeof(keyBlockTranserzone));	
						isKeyBlockTransferZoneFound = true;
						printNode(&keyBlockTranserzone, "keyBlockTranserzone");

					}
					blink(YELLOW,3);

					backSensorPanel(ENABLE);
					while(!isNode())
						line_following_reverse_pd_slow_slow();

					backSensorPanel(DISABLE);

					turnToDirection(SOUTH);

					for(int count=0; count<5; count++){
						gotoNextFrontNode();
						delay(100);
					}

					turnToDirection(EAST);

					DRY_RUN_STATE = RUN_TYPE_FINISHED;
					return 1;
				}

				else{
					if(Sensor3==WHITE){  // key Block

						keyBlockTranserzone.x = currentPosition.x;

						//write transfer zone location to EEPROM
						eeprom_write_block((const void*)&keyBlockTranserzone, (void*)100, sizeof(keyBlockTranserzone));	
						isKeyBlockTransferZoneFound = true;

						printNode(&keyBlockTranserzone, "keyBlockTranserzone");
					}
					else{

						assemblyBlockTransferZone.x = currentPosition.x;

						//write transfer zone location to EEPROM
						eeprom_write_block((const void*)&assemblyBlockTransferZone, (void*)110, sizeof(assemblyBlockTransferZone));	
						isAssemblyBlockTransferZoneFound = true;

						printNode(&assemblyBlockTransferZone, "assemblyBlockTransferZone");
					}

					blink(YELLOW, 3);
					
				} // found one transfer zone
				break;
			} // detected trnsfer zone
		}
		
		backSensorPanel(ENABLE);
		while(!isNode())
			line_following_reverse_pd_slow();

		backSensorPanel(DISABLE);
		rotateAntiClockwise180();
		//gotoNextFrontNodeUnknownDistance();
		updateState(EAST);

		gotoNextFrontNodeUnknownDistance();
		updateState(createNode(5,1));
		stop(50);
		gotoAdjacentNodeSlow(createNode(4,1));
		stop(50);
		gotoAdjacentNodeSlow(createNode(4,0));
		stop(500);

		//second place
		Serial.println("Go through second zone");

		backSensorPanel(ENABLE);

		if(isNode())
			while(isNode())
				moveForward(150,150);

		delay(10);

		beginEncoderCount = getRighttEncoderCount();
		EnCounter = getRighttEncoderCount()-beginEncoderCount;

		while(EnCounter<36){
			line_following_pd_slow_slow();

			EnCounter = getRighttEncoderCount()-beginEncoderCount;
			if(EnCounter>14 && (Sensor3==WHITE || Sensor7 == WHITE)){

				stop();
				blink(YELLOW, 1);

				if(isKeyBlockTransferZoneFound || isAssemblyBlockTransferZoneFound){ //one transfer zone is already found

					if(isKeyBlockTransferZoneFound && keyBlockTranserzone.x != currentPosition.x){
						assemblyBlockTransferZone.x = currentPosition.x;

						//write transfer zone location to EEPROM
						eeprom_write_block((const void*)&assemblyBlockTransferZone, (void*)110, sizeof(assemblyBlockTransferZone));	
						isAssemblyBlockTransferZoneFound = true;

						printNode(&assemblyBlockTransferZone, "assemblyBlockTransferZone");
					}
					else if(isAssemblyBlockTransferZoneFound && assemblyBlockTransferZone.x != currentPosition.x){
						keyBlockTranserzone.x = currentPosition.x;

						//write transfer zone location to EEPROM
						eeprom_write_block((const void*)&keyBlockTranserzone, (void*)100, sizeof(keyBlockTranserzone));	
						isKeyBlockTransferZoneFound = true;
						printNode(&keyBlockTranserzone, "keyBlockTranserzone");

					}
					blink(YELLOW,3);

					backSensorPanel(ENABLE);
					while(!isNode())
						line_following_reverse_pd_slow_slow();

					backSensorPanel(DISABLE);

					turnToDirection(SOUTH);

					for(int count=0; count<4; count++){
						gotoNextFrontNode();
						delay(100);
					}

					turnToDirection(EAST);

					DRY_RUN_STATE = RUN_TYPE_FINISHED;
					return 1;
				}

				else{
					if(Sensor3==WHITE){  // key Block

						keyBlockTranserzone.x = currentPosition.x;

						//write transfer zone location to EEPROM
						eeprom_write_block((const void*)&keyBlockTranserzone, (void*)100, sizeof(keyBlockTranserzone));	
						isKeyBlockTransferZoneFound = true;

						printNode(&keyBlockTranserzone, "keyBlockTranserzone");
					}
					else{

						assemblyBlockTransferZone.x = currentPosition.x;

						//write transfer zone location to EEPROM
						eeprom_write_block((const void*)&assemblyBlockTransferZone, (void*)110, sizeof(assemblyBlockTransferZone));	
						isAssemblyBlockTransferZoneFound = true;

						printNode(&assemblyBlockTransferZone, "assemblyBlockTransferZone");
					}

					blink(YELLOW, 3);
					
				} // found one transfer zone
				break;
			} // detected trnsfer zone
		}
		
		backSensorPanel(ENABLE);
		while(!isNode())
			line_following_reverse_pd_slow();

		backSensorPanel(DISABLE);
		rotateAntiClockwise180();
		//gotoNextFrontNodeUnknownDistance();
		updateState(EAST);

		gotoNextFrontNodeUnknownDistance();
		updateState(createNode(4,1));
		stop(50);
		gotoAdjacentNodeSlow(createNode(3,1));
		stop(50);
		gotoAdjacentNodeSlow(createNode(3,0));
		stop(500);

		//second place
		Serial.println("Go through third zone");

		backSensorPanel(ENABLE);

		beginEncoderCount = getRighttEncoderCount();
		EnCounter = getRighttEncoderCount()-beginEncoderCount;
		if(isNode())
			moveForward(150,150);
			while(isNode() && EnCounter<6){
				EnCounter = getRighttEncoderCount()-beginEncoderCount;
			}

		delay(10);

		beginEncoderCount = getRighttEncoderCount();
		EnCounter = getRighttEncoderCount()-beginEncoderCount;

		while(EnCounter<36){
			line_following_pd_slow_slow();

			EnCounter = getRighttEncoderCount()-beginEncoderCount;
			if(EnCounter>14  && (Sensor3==WHITE || Sensor7 == WHITE)){

				stop();
				blink(YELLOW, 1);

				if(isKeyBlockTransferZoneFound || isAssemblyBlockTransferZoneFound){ //one transfer zone is already found

					if(isKeyBlockTransferZoneFound && keyBlockTranserzone.x != currentPosition.x){
						assemblyBlockTransferZone.x = currentPosition.x;

						//write transfer zone location to EEPROM
						eeprom_write_block((const void*)&assemblyBlockTransferZone, (void*)110, sizeof(assemblyBlockTransferZone));	
						isAssemblyBlockTransferZoneFound = true;

						printNode(&assemblyBlockTransferZone, "assemblyBlockTransferZone");
					}
					else if(isAssemblyBlockTransferZoneFound && assemblyBlockTransferZone.x != currentPosition.x){
						keyBlockTranserzone.x = currentPosition.x;

						//write transfer zone location to EEPROM
						eeprom_write_block((const void*)&keyBlockTranserzone, (void*)100, sizeof(keyBlockTranserzone));	
						isKeyBlockTransferZoneFound = true;
						printNode(&keyBlockTranserzone, "keyBlockTranserzone");

					}
					blink(YELLOW,3);

					backSensorPanel(ENABLE);
					while(!isNode())
						line_following_reverse_pd_slow_slow();

					backSensorPanel(DISABLE);

					turnToDirection(SOUTH);

					for(int count=0; count<3; count++){
						gotoNextFrontNode();
						delay(100);
					}

					turnToDirection(EAST);

					DRY_RUN_STATE = RUN_TYPE_FINISHED;
					return 1;
				}

				else{
					if(Sensor3==WHITE){  // key Block

						keyBlockTranserzone.x = currentPosition.x;

						//write transfer zone location to EEPROM
						eeprom_write_block((const void*)&keyBlockTranserzone, (void*)100, sizeof(keyBlockTranserzone));	
						isKeyBlockTransferZoneFound = true;

						printNode(&keyBlockTranserzone, "keyBlockTranserzone");
					}
					else{

						assemblyBlockTransferZone.x = currentPosition.x;

						//write transfer zone location to EEPROM
						eeprom_write_block((const void*)&assemblyBlockTransferZone, (void*)110, sizeof(assemblyBlockTransferZone));	
						isAssemblyBlockTransferZoneFound = true;

						printNode(&assemblyBlockTransferZone, "assemblyBlockTransferZone");
					}

					blink(YELLOW, 3);
					
				}  // found one transfer zone
				break;
			} // detected trnsfer zone
		}
		
		backSensorPanel(ENABLE);
		while(!isNode())
			line_following_reverse_pd_slow();

		backSensorPanel(DISABLE);
		rotateAntiClockwise180();
		//gotoNextFrontNodeUnknownDistance();
		updateState(EAST);

		gotoNextFrontNodeUnknownDistance();
		updateState(createNode(3,1));
		stop(50);
		gotoAdjacentNodeSlow(createNode(2,1));
		stop(50);
		gotoAdjacentNodeSlow(createNode(2,0));
		stop(500);
	
		//fourth place
		Serial.println("Go through fourth zone");

		backSensorPanel(ENABLE);

		if(isNode())
			while(isNode())
				moveForward(150,150);

		delay(10);

		beginEncoderCount = getRighttEncoderCount();
		EnCounter = getRighttEncoderCount()-beginEncoderCount;

		while(EnCounter<36){
			line_following_pd_slow_slow();

			EnCounter = getRighttEncoderCount()-beginEncoderCount;
			if(EnCounter>14  && (Sensor3==WHITE || Sensor7 == WHITE)){

				stop();
				blink(YELLOW, 1);

				if(isKeyBlockTransferZoneFound || isAssemblyBlockTransferZoneFound){ //one transfer zone is already found

					if(isKeyBlockTransferZoneFound && keyBlockTranserzone.x != currentPosition.x){
						assemblyBlockTransferZone.x = currentPosition.x;

						//write transfer zone location to EEPROM
						eeprom_write_block((const void*)&assemblyBlockTransferZone, (void*)110, sizeof(assemblyBlockTransferZone));	
						isAssemblyBlockTransferZoneFound = true;

						printNode(&assemblyBlockTransferZone, "assemblyBlockTransferZone");
					}
					else if(isAssemblyBlockTransferZoneFound && assemblyBlockTransferZone.x != currentPosition.x){
						keyBlockTranserzone.x = currentPosition.x;

						//write transfer zone location to EEPROM
						eeprom_write_block((const void*)&keyBlockTranserzone, (void*)100, sizeof(keyBlockTranserzone));	
						isKeyBlockTransferZoneFound = true;
						printNode(&keyBlockTranserzone, "keyBlockTranserzone");

					}
					blink(YELLOW,3);

					backSensorPanel(ENABLE);
					while(!isNode())
						line_following_reverse_pd_slow_slow();

					backSensorPanel(DISABLE);

					turnToDirection(SOUTH);

					for(int count=0; count<2; count++){
						gotoNextFrontNode();
						delay(100);
					}

					turnToDirection(EAST);

					DRY_RUN_STATE = RUN_TYPE_FINISHED;
					return 1;
				}

				else{
					if(Sensor3==WHITE){  // key Block

						keyBlockTranserzone.x = currentPosition.x;

						//write transfer zone location to EEPROM
						eeprom_write_block((const void*)&keyBlockTranserzone, (void*)100, sizeof(keyBlockTranserzone));	
						isKeyBlockTransferZoneFound = true;

						printNode(&keyBlockTranserzone, "keyBlockTranserzone");
					}
					else{

						assemblyBlockTransferZone.x = currentPosition.x;

						//write transfer zone location to EEPROM
						eeprom_write_block((const void*)&assemblyBlockTransferZone, (void*)110, sizeof(assemblyBlockTransferZone));	
						isAssemblyBlockTransferZoneFound = true;

						printNode(&assemblyBlockTransferZone, "assemblyBlockTransferZone");
					}

					blink(YELLOW, 3);
					
				}  // found one transfer zone
				break;
			} // detected trnsfer zone
		}
		
		backSensorPanel(ENABLE);
		while(!isNode())
			line_following_reverse_pd_slow();

		backSensorPanel(DISABLE);
		rotateAntiClockwise180();
		//gotoNextFrontNodeUnknownDistance();
		updateState(EAST);

		gotoNextFrontNodeUnknownDistance();
		updateState(createNode(2,1));
		stop(50);
		gotoAdjacentNode(createNode(3,1));
		stop(50);
		gotoAdjacentNode(createNode(4,1));
		stop(50);
		gotoAdjacentNodeSlow(createNode(5,1));
		stop(50);
		gotoAdjacentNodeSlow(createNode(5,0));
		stop(500);
	}

	if(!isKeyBlockTransferZoneFound){
		while(1){
			stop();
		}
	}

	return 0;
}

void caliibrateSensorsBySensors(){
	Serial.println("Calibrating start..");
	int i;
	#define motionSpeed 120

	backSensorPanel(ENABLE);

	int beginEncoderCount = getRighttEncoderCount();
	rotateAntiClockwise(200,200);
	delay(50);

	rotateAntiClockwise(motionSpeed,motionSpeed);
	while((getRighttEncoderCount()-beginEncoderCount)<20){
		sensorPannel.calibrate();
		sensorPannelBack.calibrate();
	}

	//Serial.println(getRighttEncoderCount()-beginEncoderCount);

	beginEncoderCount = getRighttEncoderCount();

	rotateClockwise(200,200);
	delay(50);

	rotateClockwise(motionSpeed,motionSpeed);
	while((getRighttEncoderCount()-beginEncoderCount)<10){
		sensorPannel.calibrate();
		sensorPannelBack.calibrate();
	}

	while(SensorFrontLeft==BLACK || SensorFrontRight==BLACK);

	stop(100);
	//Serial.println(getRighttEncoderCount()-beginEncoderCount);

	beginEncoderCount = getRighttEncoderCount();

	rotateClockwise(200,200);
	delay(50);

	rotateClockwise(motionSpeed,motionSpeed);
	while((getRighttEncoderCount()-beginEncoderCount)<20){
		sensorPannel.calibrate();
		sensorPannelBack.calibrate();
	}

	//Serial.println(getRighttEncoderCount()-beginEncoderCount);

	beginEncoderCount = getRighttEncoderCount();

	rotateAntiClockwise(200,200);
	delay(50);

	rotateAntiClockwise(motionSpeed,motionSpeed);
	while((getRighttEncoderCount()-beginEncoderCount)<10){
		sensorPannel.calibrate();
		sensorPannelBack.calibrate();
	}

	while(SensorFrontLeft==BLACK || SensorFrontRight==BLACK);

	stop(100);
	//Serial.println(getRighttEncoderCount()-beginEncoderCount);

	backSensorPanel(DISABLE);
	Serial.println("Calibrating end..");
}



/******************   The competition run  ***************************/

void land(){
	int beginEncoderCount = getRighttEncoderCount();

	while((getRighttEncoderCount()-beginEncoderCount)<6){  
		moveForward(200,200);
	}

	while((getRighttEncoderCount()-beginEncoderCount)<12){  
		moveForward(150,150);
	}

	gotoNextFrontNodeUnknownDistance();

	updateState(EAST, createNode(0,0));
	beginEncoderCount = getRighttEncoderCount();
	while((getRighttEncoderCount()-beginEncoderCount)<2){  
		moveForward(100,100);
	}
	stop();
}

node findIntruderBox(){
	//optimalPathFinder.refreshGrid();
	node moneyBox;

	//to save result from isObjectDetected() function
	int theSensor=2;
	unsigned int distance;
	int thePreviousSensor=2;
	unsigned int previousDistance;
	
	//create optimal search path
	path optimalSearchPath;
	node optimalSearchPathNodes[(NO_OF_GRID_ROWS-1)*(NO_OF_GRID_COLUMNS-1)];
	optimalSearchPath.nodes = optimalSearchPathNodes;
	createOptimalSearchPath(&optimalSearchPath);

	//debug
	/*
	Serial.println();
	Serial.print("Search Path");
	showPath(&optimalSearchPath);
	Serial.println();*/

	//gotoAdjacentNode(createNode(0,0));
	if(isObjectDetected(&theSensor, &distance)){

		thePreviousSensor = theSensor;

		if(isObjectDetected(&theSensor, &distance)){
			//delay(30);
			if(thePreviousSensor == theSensor && isObjectDetected(&theSensor, &distance)){

				int objectDirection= currentDirection+theSensor;
				if(objectDirection==4)
					objectDirection=0;
				else if(objectDirection==-1)
					objectDirection=3;

				int shift = (distance+20)/30;

				if(shift != 0){                              // 11 12 2012

					switch(objectDirection){
						case 0: return createNode(currentPosition.x+shift, currentPosition.y);
						case 1: return createNode(currentPosition.x, currentPosition.y+shift);
						case 2: return createNode(currentPosition.x-shift, currentPosition.y);
						case 3: return createNode(currentPosition.x, currentPosition.y-shift);
					}
				}
			}
		}
	}

	turnToDirection(NORTH);
	if(isObjectDetected(&theSensor, &distance)){

		thePreviousSensor = theSensor;

		if(isObjectDetected(&theSensor, &distance)){
			//delay(30);
			if(thePreviousSensor == theSensor && isObjectDetected(&theSensor, &distance)){

				int objectDirection= currentDirection+theSensor;
				if(objectDirection==4)
					objectDirection=0;
				else if(objectDirection==-1)
					objectDirection=3;

				int shift = (distance+20)/30;

				if(shift != 0){                              // 11 12 2012

					switch(objectDirection){
						case 0: return createNode(currentPosition.x+shift, currentPosition.y);
						case 1: return createNode(currentPosition.x, currentPosition.y+shift);
						case 2: return createNode(currentPosition.x-shift, currentPosition.y);
						case 3: return createNode(currentPosition.x, currentPosition.y-shift);
					}
				}
			}
		}
	}
	delay(50);

	for(int counter0=0; counter0<optimalSearchPath.length; counter0++){

		gotoAdjacentNode(optimalSearchPath.nodes[counter0]);

		//debug
		//sendSonarReadings();
		//delay(30);
		if(isObjectDetected(&theSensor, &distance)){

			thePreviousSensor = theSensor;

			if(isObjectDetected(&theSensor, &distance)){
				if(thePreviousSensor == theSensor && isObjectDetected(&theSensor, &distance)){
					int objectDirection= currentDirection+theSensor;
					if(objectDirection==4)
						objectDirection=0;
					else if(objectDirection==-1)
						objectDirection=3;

					int shift = (distance+20)/30;

					switch(objectDirection){
						case 0: return createNode(currentPosition.x+shift, currentPosition.y);
						case 1: return createNode(currentPosition.x, currentPosition.y+shift);
						case 2: return createNode(currentPosition.x-shift, currentPosition.y);
						case 3: return createNode(currentPosition.x, currentPosition.y-shift);
					}
				}
			}
		}
		//delay(200);
	}

	optimalPathFinder.getShortestPath(currentPosition, currentDirection, createNode(0,0), &optimalSearchPath);
	for(int counter0=0; counter0<optimalSearchPath.length; counter0++){

		gotoAdjacentNode(optimalSearchPath.nodes[counter0]);

		//debug
		//sendSonarReadings();
		//delay(30);

		if(isObjectDetected(&theSensor, &distance)){
			int objectDirection= currentDirection+theSensor;
			if(objectDirection==4)
				objectDirection=0;
			else if(objectDirection==-1)
				objectDirection=3;

			int shift = (distance+20)/30;

			switch(objectDirection){
				case 0: return createNode(currentPosition.x+shift, currentPosition.y);
				case 1: return createNode(currentPosition.x, currentPosition.y+shift);
				case 2: return createNode(currentPosition.x-shift, currentPosition.y);
				case 3: return createNode(currentPosition.x, currentPosition.y-shift);
			}
		}
		//delay(200);
	}

	return createNode(0,0);
}

void depositeIntruderBox(node *intruderBoxCoordination){
	path routePath;
	node routePathNodes[(NO_OF_GRID_ROWS-1)*(NO_OF_GRID_COLUMNS-1)];
	routePath.nodes = routePathNodes;
	optimalPathFinder.getShortestPath(currentPosition, currentDirection, createNode(intruderBoxCoordination->x, intruderBoxCoordination->y), &routePath);

	/*
	printNode(&currentPosition,"Before Depotition: robot at");
	Serial.println();
	Serial.print("Path to Intruder Box: ");
	showPath(&routePath);
	Serial.println();*/

	int beginEncoderCount = getRighttEncoderCount();
	int EnCount = getRighttEncoderCount() - beginEncoderCount;
	
	                                               //routePath.length-1
	for(int counter0=0; counter0<(routePath.length-2); counter0++){ //path has two or more nodes
		turnToDirection(isAdjacent(currentPosition,routePath.nodes[counter0]));
		delay(100);
		if(isAdjacent(routePath.nodes[counter0],routePath.nodes[counter0+1])==currentDirection){

			beginEncoderCount = getRighttEncoderCount();

			while((getRighttEncoderCount() - beginEncoderCount) <30){
				line_following_pd();
			}

			while(!isNode()){
				line_following_pd();
			}
			updateState(routePath.nodes[counter0]);

			//beginEncoderCount = getRighttEncoderCount();
			while(isNode()){ //&& (getRighttEncoderCount()-beginEncoderCount)<10
				moveForward(100,100);
			}

		}
		else{
			gotoAdjacentNode(routePath.nodes[counter0]);
			delay(50);
		}
	}

	if(routePath.length>1){
		turnTowardAdjacentNode(currentPosition, routePath.nodes[routePath.length-2]);
		//gotoAdjacentNodeSlow(routePath.nodes[routePath.length-2]);
		beginEncoderCount = getRighttEncoderCount();
		while((getRighttEncoderCount()-beginEncoderCount)<35){
			line_following_pd();
		}
		gotoNextFrontNodeUnknownDistance();
		updateState(routePath.nodes[routePath.length-2]);
		stop(100);
	}

	turnTowardAdjacentNode(currentPosition, routePath.nodes[routePath.length-1]);

	beginEncoderCount = getRighttEncoderCount();
	EnCount = getRighttEncoderCount() - beginEncoderCount;
	moveForward(160,160);
	while(isNode() && EnCount<6){
		EnCount = getRighttEncoderCount() - beginEncoderCount;
	}
	
	beginEncoderCount = getRighttEncoderCount();

	while((getRighttEncoderCount()-beginEncoderCount)<31){  
		line_following_pd_slow_slow();
	}
	while((getRighttEncoderCount()-beginEncoderCount)<36){ 
		moveForward(50,50);
	}
	moveForward(20,20);
	delay(50);
	stop(); 

	//grabBox
	grabFrontBox();
	//setHandElevation(25);
	delay(50);
	//Serial.println("Box grabed! ");
	
	//4_12_2012
	if(grid[routePath.nodes[routePath.length-1].x][routePath.nodes[routePath.length-1].y]){  //in front of allowed node

		//Serial.println("Deposite:In front of allowed node");
		if(routePath.nodes[routePath.length-1].y == 0 && routePath.nodes[routePath.length-1].x > 1 && currentDirection == WEST){ 

			//Serial.println("Deposite:Danger front; transfer zone");
			gotoReverseNodeUnknownDistance();
			stop(50);
		}
		else{
			//Serial.println("Deposite:front is clear");
			gotoNextFrontNodeUnknownDistance();
			updateState(currentDirection,routePath.nodes[routePath.length-1]);
		}

		//route to deposition area
		optimalPathFinder.getShortestPath(currentPosition, currentDirection, createNode(1, 0), &routePath);

	}else{
		//Serial.println("Deposite:In front of blocked node");
		gotoReverseNodeUnknownDistance();
		stop(50);
		optimalPathFinder.getShortestPath(currentPosition, currentDirection, createNode(1, 0), &routePath);
	}

	//debug
	/*
	printNode(&currentPosition, "Robot at");
	
	Serial.println();
	Serial.print("Path to Deposition Area: ");
	showPath(&routePath);
	Serial.println();
	*/

	for(int counter0=0; counter0<(routePath.length-1); counter0++){

		//check if node.y=0; then may be robot should reverse
		//printNode(&routePath.nodes[counter0],"Next node");
		if(routePath.nodes[counter0].y == 0 && routePath.nodes[counter0].x > 1  && currentPosition.y==1){

			//Serial.println("Danger node");
			
			turnTowardAdjacentNode(currentPosition, routePath.nodes[counter0]);

			beginEncoderCount = getRighttEncoderCount();
			while((getRighttEncoderCount()-beginEncoderCount)<35){
				line_following_pd_slow();
			}

			stop(200);

			beginEncoderCount = getRighttEncoderCount();
			while((getRighttEncoderCount()-beginEncoderCount)<6){
				rotateClockwise(150,150);
			}

			while(Sensor5==WHITE){
				moveForward(200,200);
			}

			while(Sensor5==BLACK){
				moveForward(200,200);
			}

			while(SensorFrontLeft==BLACK || SensorFrontRight==BLACK){
				rotateClockwise(120,120);
			}
			gotoNextFrontNodeUnknownDistance();
			counter0++;

			beginEncoderCount = getRighttEncoderCount();
			while((getRighttEncoderCount()-beginEncoderCount)<4){
				moveForward(100,100);
			}

			stop(100);

			updateState(SOUTH, routePath.nodes[counter0]);
			

		}
		else{
			gotoAdjacentNodeSlow(routePath.nodes[counter0]);
			delay(10);
		}
	}
	if(!(currentPosition.x==1 && currentPosition.y==0)){
		turnTowardAdjacentNode(currentPosition, routePath.nodes[routePath.length-1]);
		gotoNextFrontNodeUnknownDistance();
		updateState(createNode(1,0));
	}

	//printNode(&currentPosition, "Robot at");

	turnToDirection(WEST);
	delay(50);
	//setHandElevation(0);
	beginEncoderCount = getRighttEncoderCount();
	while((getRighttEncoderCount()-beginEncoderCount)<14){
		line_following_pd_slow();
	}
	stop();
	//delay(500);
	//setHandElevation(0);
	releaseFrontBox();
	//delay(500);

	backSensorPanel(ENABLE);
	beginEncoderCount = getRighttEncoderCount();
	EnCount = getRighttEncoderCount() - beginEncoderCount;
	while(!isNode() && EnCount<14){
		line_following_reverse_pd_slow_slow();
		EnCount = getRighttEncoderCount() - beginEncoderCount;
	}

	backSensorPanel(DISABLE);

	stop(50);

	//Serial.println("Box deposited! ");
}

void waitForTheKey(){
	path pathToTransferZone;
	node transferZonePathNodes[(NO_OF_GRID_ROWS-1)*(NO_OF_GRID_COLUMNS-1)];
	pathToTransferZone.nodes = transferZonePathNodes;
	optimalPathFinder.getShortestPath(currentPosition, currentDirection, createNode(keyBlockTranserzone.x, 0), &pathToTransferZone);

	//debug
	/*
	Serial.println();
	Serial.print("Path to tansfer zone");
	showPath(&pathToTransferZone);
	Serial.println();*/
	
	//rotateAntiClockwise90();
	//updateState(NORTH, currentPosition);

	for(int counter0=0; counter0<(pathToTransferZone.length-1); counter0++){

		/*
		gotoAdjacentNode(pathToTransferZone.nodes[counter0]);
		delay(100);
		*/
		if(pathToTransferZone.nodes[counter0].y == 0 && pathToTransferZone.nodes[counter0].x > 1 && pathToTransferZone.nodes[counter0].x != keyBlockTranserzone.x && currentPosition.y==1){
			
			if(isAdjacent(pathToTransferZone.nodes[counter0], pathToTransferZone.nodes[counter0+1]) == SOUTH){  //path goes to south direction

				
				int beginEncoderCount = getRighttEncoderCount();
				while((getRighttEncoderCount()-beginEncoderCount)<27){
					line_following_pd();
				}

				stop(200);

				beginEncoderCount = getRighttEncoderCount();
				while((getRighttEncoderCount()-beginEncoderCount)<6){
					rotateClockwise(200,200);
				}

				while(Sensor5==WHITE){
					moveForward(200,200);
				}

				while(Sensor5==BLACK){
					moveForward(200,200);
				}

				while(SensorFrontLeft==BLACK || SensorFrontRight==BLACK){
					rotateClockwise(200,200);
				}
				stop(100);

				gotoNextFrontNode();
				stop(100);
				updateState(SOUTH,pathToTransferZone.nodes[counter0+1]);
				counter0++;
			}

			else if(isAdjacent(pathToTransferZone.nodes[counter0], pathToTransferZone.nodes[counter0+1]) == NORTH){  //path goes to north

				int beginEncoderCount = getRighttEncoderCount();
				while((getRighttEncoderCount()-beginEncoderCount)<27){
					line_following_pd();
				}

				stop(200);

				beginEncoderCount = getRighttEncoderCount();
				while((getRighttEncoderCount()-beginEncoderCount)<6){
					rotateAntiClockwise(200,200);
				}

				while(Sensor5==WHITE){
					moveForward(200,200);
				}

				while(Sensor5==BLACK){
					moveForward(200,200);
				}

				while(SensorFrontLeft==BLACK || SensorFrontRight==BLACK){
					rotateAntiClockwise(200,200);
				}
				stop(100);

				gotoNextFrontNode();
				stop(100);
				updateState(NORTH,pathToTransferZone.nodes[counter0+1]);
				counter0++;
			}
			/*
			gotoAdjacentNodeSlow(pathToTransferZone.nodes[counter0]);
			delay(100);
			*/
		}
		else{
			gotoAdjacentNode(pathToTransferZone.nodes[counter0]);
			delay(100);
		}
	}

	gotoAdjacentNodeSlow(pathToTransferZone.nodes[pathToTransferZone.length-1]);

	stop(100);
	
	turnToDirection(EAST);

	stop(100);

	int beginEncoderCount = getRighttEncoderCount();

	while((getRighttEncoderCount()-beginEncoderCount)<35){  
		line_following_pd_slow_slow();
	}

	stop(100);

	rotateAntiClockwise90bySensors();

	stop(100);

	gotoNextFrontNodeUnknownDistance();

	updateState(WEST, currentPosition);

	stop(100);

	unsigned int distance;

	//Serial.println("Waiting for the key Box");
	while(1){
		distance = getFrontSonarReading();

		if(0<distance && distance<5){
			delay(30);
			distance = getFrontSonarReading();

			if(0<distance && distance<5){
				delay(30);
				distance = getFrontSonarReading();

				if(0<distance && distance<5){
					delay(30);
					distance = getFrontSonarReading();

					if(0<distance && distance<5)
						break;
				}
			}
		}
		delay(500);
	}

	Serial.println("Key Box at transfer zone");
	blink(RED, 2);
	delay(WAITING_TIME_FOR_THE_BOXES);
}

void unlockTheSecureZone(){

	stop(500);

	grabFrontBox();
	//delay(1000);

	path pathToUnlockZone;
	node unlockZonePathNodes[(NO_OF_GRID_ROWS-1)*(NO_OF_GRID_COLUMNS-1)];
	pathToUnlockZone.nodes = unlockZonePathNodes;
	
	//find a path
	optimalPathFinder.getShortestPath(currentPosition, currentDirection, createNode(5, 3), &pathToUnlockZone);

	//debug
	/*
	Serial.println();
	Serial.print("Path to tansfer zone");
	showPath(&pathToUnlockZone);
	Serial.println();
	*/

	//go to unlock zone
	for(int counter0=0; counter0<pathToUnlockZone.length; counter0++){
		gotoAdjacentNode(pathToUnlockZone.nodes[counter0]);
		delay(50);
	}

	turnToDirection(NORTH);
	delay(50);

	int beginEncoderCount = getRighttEncoderCount();
	while((getRighttEncoderCount()-beginEncoderCount)<70){
		line_following_pd();
	}
	
	gotoNextFrontNodeUnknownDistance();
	delay(50);
	rotateAntiClockwise90bySensors();
	delay(100);


	beginEncoderCount = getRighttEncoderCount();
	while((getRighttEncoderCount()-beginEncoderCount)<16){
		line_following_pd_slow_slow();
	}
	stop(100);

	releaseFrontBox();

	stop();
	gotoReverseNodeUnknownDistance();
	beginEncoderCount = getRighttEncoderCount();
	while((getRighttEncoderCount()-beginEncoderCount)<4){
		reverse(100,100);
	}
	delay(50);
	rotateAntiClockwise90bySensors();
	delay(50);

	beginEncoderCount = getRighttEncoderCount();
	while((getRighttEncoderCount()-beginEncoderCount)<70){
		line_following_pd();
	}
	
	gotoNextFrontNodeUnknownDistance();
	stop(50);
	updateState(SOUTH,createNode(5,3));
}

void waitForTheAssemblyBox(){

	path pathToTransferZone;
	node transferZonePathNodes[(NO_OF_GRID_ROWS-1)*(NO_OF_GRID_COLUMNS-1)];
	pathToTransferZone.nodes = transferZonePathNodes;
	optimalPathFinder.getShortestPath(currentPosition, currentDirection, createNode(assemblyBlockTransferZone.x, 0), &pathToTransferZone);

	//debug
	/*
	Serial.println();
	Serial.print("Path to tansfer zone");
	showPath(&pathToTransferZone);
	Serial.println();
	*/
	
	//rotateAntiClockwise90();
	//updateState(NORTH, currentPosition);

	for(int counter0=0; counter0<(pathToTransferZone.length-1); counter0++){

		/*
		gotoAdjacentNode(pathToTransferZone.nodes[counter0]);
		delay(100);
		*/
		if(pathToTransferZone.nodes[counter0].y == 0 && pathToTransferZone.nodes[counter0].x > 1 && pathToTransferZone.nodes[counter0].x != keyBlockTranserzone.x && currentPosition.y==1){
			
			if(isAdjacent(pathToTransferZone.nodes[counter0], pathToTransferZone.nodes[counter0+1]) == SOUTH){  //path goes to south direction

				
				int beginEncoderCount = getRighttEncoderCount();
				while((getRighttEncoderCount()-beginEncoderCount)<27){
					line_following_pd();
				}

				stop(200);

				beginEncoderCount = getRighttEncoderCount();
				while((getRighttEncoderCount()-beginEncoderCount)<6){
					rotateClockwise(200,200);
				}

				while(Sensor5==WHITE){
					moveForward(200,200);
				}

				while(Sensor5==BLACK){
					moveForward(200,200);
				}

				while(SensorFrontLeft==BLACK || SensorFrontRight==BLACK){
					rotateClockwise(200,200);
				}
				stop(100);

				gotoNextFrontNode();
				stop(100);
				updateState(SOUTH,pathToTransferZone.nodes[counter0+1]);
				counter0++;
			}

			else if(isAdjacent(pathToTransferZone.nodes[counter0], pathToTransferZone.nodes[counter0+1]) == NORTH){  //path goes to north

				int beginEncoderCount = getRighttEncoderCount();
				while((getRighttEncoderCount()-beginEncoderCount)<27){
					line_following_pd();
				}

				stop(200);

				beginEncoderCount = getRighttEncoderCount();
				while((getRighttEncoderCount()-beginEncoderCount)<6){
					rotateAntiClockwise(200,200);
				}

				while(Sensor5==WHITE){
					moveForward(200,200);
				}

				while(Sensor5==BLACK){
					moveForward(200,200);
				}

				while(SensorFrontLeft==BLACK || SensorFrontRight==BLACK){
					rotateAntiClockwise(200,200);
				}
				stop(100);

				gotoNextFrontNode();
				stop(100);
				updateState(NORTH,pathToTransferZone.nodes[counter0+1]);
				counter0++;
			}
			/*
			gotoAdjacentNodeSlow(pathToTransferZone.nodes[counter0]);
			delay(100);
			*/
		}
		else{
			gotoAdjacentNode(pathToTransferZone.nodes[counter0]);
			delay(100);
		}
	}

	gotoAdjacentNodeSlow(pathToTransferZone.nodes[pathToTransferZone.length-1]);

	stop(100);
	
	turnToDirection(EAST);

	stop(100);

	int beginEncoderCount = getRighttEncoderCount();

	while((getRighttEncoderCount()-beginEncoderCount)<35){  
		line_following_pd_slow_slow();
	}

	stop(100);

	rotateAntiClockwise90bySensors();

	stop(100);

	gotoNextFrontNodeUnknownDistance();

	updateState(WEST, currentPosition);

	stop(100);

	unsigned int distance;

	while(1){
		distance = getFrontSonarReading();

		if(0<distance && distance<5){
			delay(30);
			distance = getFrontSonarReading();

			if(0<distance && distance<5){
				delay(30);
				distance = getFrontSonarReading();

				if(0<distance && distance<5){
					delay(30);
					distance = getFrontSonarReading();

					if(0<distance && distance<5)
						break;
				}
			}
		}
		delay(500);
	}

	//Serial.println("assembly Box at transfer zone");
	blink(RED, 2);
	delay(WAITING_TIME_FOR_THE_BOXES);
}

void saveTheEarth(){

	stop(500);

	grabFrontBox();
	//delay(1000);

	path pathGoal;
	node goalPathNodes[(NO_OF_GRID_ROWS-1)*(NO_OF_GRID_COLUMNS-1)];
	pathGoal.nodes = goalPathNodes;
	
	//find a path
	optimalPathFinder.getShortestPath(currentPosition, currentDirection, createNode(5, 3), &pathGoal);

	//debug
	/*
	Serial.println();
	Serial.print("Path to the goal enterance");
	showPath(&pathGoal);
	Serial.println();
	*/

	//go to the goal enterance
	for(int counter0=0; counter0<(pathGoal.length-1); counter0++){
		turnToDirection(isAdjacent(currentPosition,pathGoal.nodes[counter0]));
		delay(100);
		if(isAdjacent(pathGoal.nodes[counter0],pathGoal.nodes[counter0+1])==currentDirection){
			while(!isNode()){
				line_following_pd();
			}
			updateState(pathGoal.nodes[counter0]);

			int beginEncoderCount = getRighttEncoderCount();
			while(isNode() && (getRighttEncoderCount()-beginEncoderCount)<10){
				moveForward(100,100);
			}
		}
		else{
			gotoAdjacentNode(pathGoal.nodes[counter0]);
			delay(50);
		}
	}
	gotoAdjacentNode(pathGoal.nodes[pathGoal.length-1]);
	
	updateState(createNode(5,3));

	turnToDirection(NORTH);
	delay(50);

	int beginEncoderCount = getRighttEncoderCount();
	while((getRighttEncoderCount()-beginEncoderCount)<80){
		line_following_pd();
	}
	
	gotoNextFrontNodeUnknownDistance();
	delay(50);
	rotateClockwise90bySensors();
	delay(100);

	beginEncoderCount = getRighttEncoderCount();
	while((getRighttEncoderCount()-beginEncoderCount)<25){
		line_following_pd();
	}

	setHandElevation(25);

	while((getRighttEncoderCount()-beginEncoderCount)<80){
		line_following_pd();
	}

	setHandElevation(0);

	while((getRighttEncoderCount()-beginEncoderCount)<360){ //360        //37 + 82.5 + 60 = 30*6 = 180       30 --> 33/63    28(C)--> 64
		line_following_pd();
	}

	beginEncoderCount = getRighttEncoderCount();
	while(SensorMiddleLeft==BLACK || SensorMiddleRight==BLACK){
		line_following_pd_slow();
	}

	stop(100);

	beginEncoderCount = getRighttEncoderCount();
	while((getRighttEncoderCount()-beginEncoderCount)<14){
		line_following_reverse_pd_slow();
	}

	stop();

	releaseFrontBox();

	stop();
}

void finish(){
	if(digitalRead(DISPLAY_PIN) == LOW)
		digitalWrite(DISPLAY_PIN,HIGH);
	
	blink(RED, 1);
	blink(YELLOW, 1);
	blink(BLUE,1);
	blink(RED, 1);
	blink(YELLOW, 1);
	blink(BLUE,1);

	digitalWrite(DISPLAY_PIN,LOW);
}

/***********************************************************/



//good
void gotoReverseNodeUnknownDistance(){

	backSensorPanel(ENABLE);
	if(SensorBack1==WHITE || SensorBack6==WHITE)
		while(SensorBack1==WHITE || SensorBack6==WHITE){
			line_following_reverse_pd_slow();
		}


	while(SensorBack1==BLACK || SensorBack6==BLACK){
		line_following_reverse_pd_slow();
	}
	stop();

	int beginEncoderCount = getRighttEncoderCount();
	while((getRighttEncoderCount()-beginEncoderCount)<6){
		reverse(130,130);
	}

	stop(50);
	backSensorPanel(DISABLE);
}

//not good
void gotoReverseNode(){

	backSensorPanel(ENABLE);
	if(SensorBack1==WHITE || SensorBack6==WHITE)
		while(SensorBack1==WHITE || SensorBack6==WHITE){
			line_following_reverse_pd_slow();
		}

	int beginEncoderCount = getRighttEncoderCount();
	while((getRighttEncoderCount()-beginEncoderCount)<25){
		line_following_reverse_pd();
	}

	gotoReverseNodeUnknownDistance();
}

#endif