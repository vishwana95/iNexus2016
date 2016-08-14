// Authors: Diunuge Buddhika Wijesinghe
// Competition iNexus 2013
// version: 3.0v: Not Optimized
// Last mod: 29-12-2012
// Comment: updated for finals


#ifndef diuStatus_H
#define diuStatus_H


#define RUN_TYPE_UNAVAILABLE 0
#define RUN_TYPE_STARTED 1
#define RUN_TYPE_FINISHED 2

#define NODE_NOT_DEFINED -1
#define NODE_IS_ALLOWED 1
#define NODE_IS_BLOCKED 0

#define DRY_RUN_SELECTION_PIN 17
#define SELECTION_PIN_0 16
#define SELECTION_PIN_1 15
#define SELECTION_PIN_2 14 

#define LANDING_DETECTION_PIN 8

#define DRY_RUN_SELECTION_SWITCH 0
#define SELECTION_SWITCH_0 1
#define SELECTION_SWITCH_1 2
#define SELECTION_SWITCH_2 3

#define OFF HIGH
#define ON LOW

int DRY_RUN_STATE = RUN_TYPE_UNAVAILABLE;
int ACTIVE_RUN_STATE = RUN_TYPE_UNAVAILABLE;

boolean _testing;
boolean _dryRun;

/*!
 * @var         currentDirection
 * @abstract    Indicate current bot direction.
 * @discussion  Stores direction of the autonomus robot which used to navigate through grid
*/
int currentDirection;

/*!
 * @var         currentPosition
 * @abstract    Indicate current bot position.
 * @discussion  Stores position of the autonomus robot using node datatype which used to navigate through grid
*/
node currentPosition;


node keyBlockTranserzone;
node assemblyBlockTransferZone;

bool isKeyBlockTransferZoneFound;
bool isAssemblyBlockTransferZoneFound;


/*!
 * @var         grid
 * @abstract    Status of the grid nodes.
 * @discussion  Stores data about status of the grid nodes.
 *              whether is it blocked or not.
*/
int grid[NO_OF_GRID_ROWS-1][NO_OF_GRID_COLUMNS-1] = {-1};

/************************** function prototypes ************************/

void initializeStatus();
boolean isDryRun();
void setDryRun(boolean state);
boolean isTesting();
void setTest(boolean state);
void updateState(int direction, node newPosition);

/************************** function definitions ************************/

void initializeStatus(){
	
	currentDirection = 1;

	currentPosition.x = 0;
	currentPosition.y = -1;

	keyBlockTranserzone.x = 2;
	keyBlockTranserzone.y = 0;
	assemblyBlockTransferZone.x = 4;
	assemblyBlockTransferZone.y = 0;

	isKeyBlockTransferZoneFound = false;
	isAssemblyBlockTransferZoneFound = false; 

	setTest(false);
	setDryRun(true);

	pinMode(DRY_RUN_SELECTION_PIN, INPUT);
	pinMode(SELECTION_PIN_0, INPUT);
	pinMode(SELECTION_PIN_1, INPUT);
	pinMode(SELECTION_PIN_2, INPUT);
	pinMode(LANDING_DETECTION_PIN, INPUT);
}

int getSwitchStatus(int Sensor){

	switch(Sensor){

		case 0:
			return digitalRead(DRY_RUN_SELECTION_PIN);
		case 1:
			return digitalRead(SELECTION_PIN_0);
		case 2:
			return digitalRead(SELECTION_PIN_1);
		case 3:
			return digitalRead(SELECTION_PIN_2);
		default:
			return LOW;
	}
}

boolean isDryRun(){
	//read external signal: Probably from a switch
	return _dryRun;
}

void setDryRun(boolean state){
	//_dryRun = state;
	if(digitalRead(DRY_RUN_SELECTION_PIN)==ON)
		_dryRun = true;
	else
		_dryRun = false;
}

boolean isTesting(){
	//read external signal: Probably from a switch
	return _testing;
}

void setTest(boolean state){
	//_testing = state;
	if(digitalRead(SELECTION_PIN_2)==ON)
		_testing = true;
	else
		_testing = false;
}

void updateState(int direction, node newPosition){
	currentDirection = direction;
	currentPosition = newPosition;
}

void updateState(int direction){
	currentDirection = direction;
}

void updateState(node newPosition){
	currentPosition = newPosition;
}

#endif