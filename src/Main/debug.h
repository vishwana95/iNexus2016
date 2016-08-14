// Authors: Diunuge Buddhika Wijesinghe
// Competition iNexus 2013
// version: 2.0v: Not Optimized
// Last mod: 11-11-2012


#ifndef diuDebug_H
#define diuDebug_H

#include "motion.h"
#include "sensorPanel.h"
#include "status.h"
#include "dataTypes.h"
#include "configuration.h"
#include "sensorPanel.h"
#include "diuSonar.h"
#include "algorithm.h"

/************************** function prototypes ************************/

//void sendSensorStatus();
//void showPath(path aPath);


/************************** function definitions ************************/

void sendSensorStatus(){
	  backSensorPanel(ENABLE);
      if( Sensor1 == BLACK){
		  Serial.write("1");
      }
      else if( Sensor1 == WHITE){
            Serial.write("0");
      }

      if( Sensor2 == BLACK){
            Serial.write("1");
      }
      else if( Sensor2 == WHITE){
            Serial.write("0");
      }

      if( Sensor3 == BLACK){
            Serial.write("1");
      }
      else if( Sensor3 == WHITE){
            Serial.write("0");
      }

      if( Sensor4 == BLACK){
            Serial.write("1");
      }
      else if( Sensor4 == WHITE){
            Serial.write("0");
      }

      if( Sensor5 == BLACK){
            Serial.write("1");
      }
      else if( Sensor5 == WHITE){
            Serial.write("0");
      }

      if( Sensor6 == BLACK){
            Serial.write("1");
      }
      else if( Sensor6 == WHITE){
            Serial.write("0");
      }

      if( Sensor7 == BLACK){
            Serial.write("1");
      }
      else if( Sensor7 == WHITE){
            Serial.write("0");
      }

      if( Sensor8 == BLACK){
            Serial.write("1");
      }
      else if( Sensor8 == WHITE){
            Serial.write("0");
      }

      if( Sensor9 == BLACK){
            Serial.write("1");
      }
      else if( Sensor9 == WHITE){
            Serial.write("0");
      }
	  Serial.write(" ");

      if( Scout == BLACK){
            Serial.write("1");
      }
      else if( Scout == WHITE){
            Serial.write("0");
      }
	  Serial.write(" ");

	  if( SensorMiddleLeft == BLACK){
            Serial.write("1");
      }
      else if( SensorMiddleLeft == WHITE){
            Serial.write("0");
      }

	  if( SensorMiddleRight == BLACK){
            Serial.write("1");
      }
      else if( SensorMiddleRight == WHITE){
            Serial.write("0");
      }

	  Serial.write(" ");

	  if( SensorFrontLeft == BLACK){
            Serial.write("1");
      }
      else if( SensorFrontLeft == WHITE){
            Serial.write("0");
      }

	  if( SensorFrontRight == BLACK){
            Serial.write("1");
      }
      else if( SensorFrontRight == WHITE){
            Serial.write("0");
      }
	  
	  Serial.write(" ");

	  if( SensorBack1 == BLACK){
		  Serial.write("1");
      }
	  else if( SensorBack1 == WHITE){
            Serial.write("0");
      }

      if( SensorBack2 == BLACK){
            Serial.write("1");
      }
      else if( SensorBack2 == WHITE){
            Serial.write("0");
      }

      if( SensorBack3 == BLACK){
            Serial.write("1");
      }
      else if( SensorBack3 == WHITE){
            Serial.write("0");
      }

      if( SensorBack4 == BLACK){
            Serial.write("1");
      }
      else if( SensorBack4 == WHITE){
            Serial.write("0");
      }

      if( SensorBack5 == BLACK){
            Serial.write("1");
      }
      else if( SensorBack5 == WHITE){
            Serial.write("0");
      }

      if( SensorBack6 == BLACK){
            Serial.write("1");
      }
      else if( SensorBack6 == WHITE){
            Serial.write("0");
      }
	  backSensorPanel(DISABLE);
	  
	  Serial.println();
}

void printNode(node *aNode, char *msg){

	Serial.println();
	Serial.print(msg);
	Serial.print("  : (");
	Serial.print(aNode->x);
	Serial.print(",");
	Serial.print(aNode->y);
	Serial.print(")");
}

void sendGridStatus(){
	Serial.println("");
	Serial.println("Grid:");
	for(int counter0=0; counter0<(NO_OF_GRID_ROWS-1); counter0++){
		for(int counter1=0; counter1<(NO_OF_GRID_COLUMNS-1); counter1++){
			
			Serial.print(grid[counter0][counter1]);
			Serial.print(" ");
		}
		Serial.println("");
	}
}

void sendSensorStatusAnalog(){
	backSensorPanel(ENABLE);
	unsigned int val;
	val = analogRead(Sensor1_PIN);
	Serial.print("1: ");
	Serial.print(val);
	Serial.print("  2: ");
	val = analogRead(Sensor2_PIN);
	Serial.print(val);
	Serial.print("  3: ");
	val = analogRead(Sensor3_PIN);
	Serial.print(val);
	Serial.print("  4: ");
	val = analogRead(Sensor4_PIN);
	Serial.print(val);
	Serial.print("  5: ");
	val = analogRead(Sensor5_PIN);
	Serial.print(val);
	Serial.print("  6: ");
	val = analogRead(Sensor6_PIN);
	Serial.print(val);
	Serial.print("  7: ");
	val = analogRead(Sensor7_PIN);
	Serial.print(val);
	Serial.print("  8: ");
	val = analogRead(Sensor8_PIN);
	Serial.print(val);
	Serial.print("  9: ");
	val = analogRead(Sensor9_PIN);
	Serial.print(val);
	Serial.print("   Scout: ");
	val = analogRead(Scout_PIN);
	Serial.print(val);

	//Serial.println();
	Serial.print("     Back  1: ");
	val = analogRead(SensorBack1_PIN);
	Serial.print(val);
	Serial.print("  2: ");
	val = analogRead(SensorBack2_PIN);
	Serial.print(val);
	Serial.print("  3: ");
	val = analogRead(SensorBack3_PIN);
	Serial.print(val);
	Serial.print("  4: ");
	val = analogRead(SensorBack4_PIN);
	Serial.print(val);
	Serial.print("  5: ");
	val = analogRead(SensorBack5_PIN);
	Serial.print(val);
	Serial.print("  6: ");
	val = analogRead(SensorBack6_PIN);
	Serial.print(val);
	backSensorPanel(DISABLE);
	Serial.println();
}

void sendSonarReadings(){

	delay(30); //recovery time

	unsigned int uS = sonarLeft.ping(); // Send ping, get ping time in microseconds (uS).
	Serial.print("Left: ");
	Serial.print(uS / US_ROUNDTRIP_CM); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
	Serial.print("cm");

	uS = sonarFront.ping(); // Send ping, get ping time in microseconds (uS).
	Serial.print("      Front: ");
	Serial.print(uS / US_ROUNDTRIP_CM); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
	Serial.print("cm");

	uS = sonarRight.ping(); // Send ping, get ping time in microseconds (uS).
	Serial.print("      Right: ");
	Serial.print(uS / US_ROUNDTRIP_CM); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
	Serial.println("cm");

}

void createAGrid(){
	/*
	grid = {{1, 1, 0, 1},
	        {1, 1, 1, 1},
			{1, 1, 1, 1},
	        {1, 1, 1, 1}};*/
	
	
	grid[0][0] = NODE_IS_ALLOWED;
	grid[0][1] = NODE_IS_BLOCKED;
	grid[0][2] = NODE_IS_ALLOWED;
	grid[0][3] = NODE_IS_ALLOWED;
	grid[1][1] = NODE_IS_ALLOWED;
	grid[1][3] = NODE_IS_ALLOWED;
	grid[3][0] = NODE_IS_BLOCKED;
	grid[3][2] = NODE_IS_BLOCKED;
}

void debugCreateOptimalSearchPath(){
	path optimalSearchPath;
	node optimalSearchPathNodes[(NO_OF_GRID_ROWS-1)*(NO_OF_GRID_COLUMNS-1)];
	optimalSearchPath.nodes = optimalSearchPathNodes;
	createOptimalSearchPath(&optimalSearchPath);
	showPath(&optimalSearchPath);
}


#endif