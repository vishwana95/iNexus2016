#ifndef diuOptimalPath_H
#define diuOptimalPath_H

#define NO_OF_NODES (NO_OF_GRID_ROWS-1)*(NO_OF_GRID_COLUMNS-1)*4

#define BLOCKED 0
#define UNLOCKED 1

#define NOT_REACHABLE 253
#define NO_PREDECESSOR 254
#define VISITED 1
#define NOT_VISITED 0

#define DIJKSTRA_INFINITY 255
#define OUT_DEGREE_MAX 3

#define TURNING_COST 1
#define NODE_TRAVEL_COST 3

#include <wprogram.h>
#include "dataTypes.h"
#include "configuration.h"
#include "status.h"

int minimalGrid[(NO_OF_GRID_ROWS-1)*(NO_OF_GRID_COLUMNS-1)]={0};

void createMinimalGrid(){

	for(int rows=0; rows<(NO_OF_GRID_ROWS-1); rows++)
		for(int columns=0; columns<(NO_OF_GRID_COLUMNS-1); columns++){
			minimalGrid[rows*(NO_OF_GRID_COLUMNS-1)+columns] = grid[rows][columns];
		}
}

class Dijkstra{
	private:
		unsigned int nodesInGraph[NO_OF_NODES];

		unsigned int distance[NO_OF_NODES];
		unsigned int predecessor[NO_OF_NODES];
		unsigned int nodeState[NO_OF_NODES];
		unsigned int nodeLevel[NO_OF_NODES];
	
	private:
		int initialileSingleSource(unsigned int startNode);
		unsigned int getWeight(unsigned int u, unsigned int v);
		void relax(unsigned int u, unsigned int v);
		unsigned int getNumberOfOutgoingConnections(unsigned int aNode, unsigned int* nodeSet);
		unsigned int extractMinNonVisited();
		bool isGoalNode(unsigned int currrentNode, unsigned int goalInSimpleCoordinates);
		unsigned int getConnectedNode(unsigned int index);
		
	public:
		Dijkstra();
		Dijkstra(int* grid);
		//~Dijkstra();
		bool getShortestPath(unsigned int startNodeInSimpleCoordinate, unsigned int currentDirection, unsigned int goalNodeInSimpleCoordinate, path *optimalPath);
		bool getShortestPath(node startNodeInSimpleCoordinate, unsigned int currentDirection, node goalNodeInSimpleCoordinate, path *optimalPath);
		void refreshGrid();
		void printNodesStatus();
		void printStatus();
		
	public: //for debug
		//unsigned int getNumberOfOutgoingConnections(unsigned int aNode, unsigned int* nodeSet);
		//unsigned int getWeight(unsigned int u, unsigned int v);
		
};

Dijkstra::Dijkstra(){

	createMinimalGrid();

	for(int i=0; i<NO_OF_NODES; i++){
		nodesInGraph[i] = minimalGrid[i/4];
	}
}

Dijkstra::Dijkstra(int* grid){
	for(int i=0; i<NO_OF_NODES; i++){
		nodesInGraph[i] = minimalGrid[i/4];
	}
}

void Dijkstra::refreshGrid(){
	createMinimalGrid();

	for(int i=0; i<NO_OF_NODES; i++){
		nodesInGraph[i] = minimalGrid[i/4];
	}
}

int Dijkstra::initialileSingleSource(unsigned int startNode){
	int node = startNode;
	for(int i=0; i<NO_OF_NODES; i++){
		distance[i] = NOT_REACHABLE;
		predecessor[i] = NO_PREDECESSOR;
		nodeState[i] = NOT_VISITED;
		nodeLevel[i] = NOT_REACHABLE;
	}
	distance[startNode] = 0;
	nodeLevel[startNode] = 0;
}

unsigned int Dijkstra::extractMinNonVisited(){
	
	int selectedNode = -1;
	int minimalDistance = DIJKSTRA_INFINITY;
	for(int currentNode=0; currentNode<NO_OF_NODES; currentNode++){
		
		if(nodeState[currentNode]==NOT_VISITED){
			
			if(distance[currentNode]<minimalDistance){
				selectedNode = currentNode;
				minimalDistance = distance[currentNode];
			}
		}
	}
	return selectedNode;
}

unsigned int Dijkstra::getNumberOfOutgoingConnections(unsigned int aNode, unsigned int* nodeSet){
	
	int rows = (NO_OF_GRID_ROWS-1);
	int columns = (NO_OF_GRID_COLUMNS-1)*4;
	
	for(int i=0; i<OUT_DEGREE_MAX; i++)
		nodeSet[i]=DIJKSTRA_INFINITY;
	
	if(aNode>= rows*columns){ //out of bound
		return 0;
	}
	else if(aNode<4){ //first node set of first row
		if(nodesInGraph[aNode]==UNLOCKED){
			switch(aNode%4){
				case 0: nodeSet[0]=1; nodeSet[1]=3; nodeSet[2]=4; return 3; //aNode+4
				case 1: nodeSet[0]=2; nodeSet[1]=0; nodeSet[2]=1+columns; return 3;  //aNode+columns
				case 2: nodeSet[0]=3; nodeSet[1]=1; return 2;
				case 3: nodeSet[0]=0; nodeSet[1]=2; return 2;
			}
		}
		else{
			switch(aNode%4){
				case 0: nodeSet[0]=aNode+4; return 1;
				case 1: nodeSet[0]=aNode+columns; return 1;
				case 2: return 0;
				case 3: return 0;
			}
		}
	}
	else if(aNode<(columns-4)){ //intermediate node set of first row
		if(nodesInGraph[aNode]==UNLOCKED){
			switch(aNode%4){
				case 0: nodeSet[0]=aNode+1; nodeSet[1]=aNode+3; nodeSet[2]=aNode+4; return 3;
				case 1: nodeSet[0]=aNode+1; nodeSet[1]=aNode-1; nodeSet[2]=aNode+columns; return 3;
				case 2: nodeSet[0]=aNode+1; nodeSet[1]=aNode-1; nodeSet[2]=aNode-4; return 3;
				case 3: nodeSet[0]=aNode-3; nodeSet[1]=aNode-1; return 2;
			}
		}
		else{
			switch(aNode%4){
				case 0: nodeSet[0]=aNode+4; return 1;
				case 1: nodeSet[0]=aNode+columns; return 1;
				case 2: nodeSet[0]=aNode-4; return 1;
				case 3: return 0;
			}
		}
	}
	else if(aNode<columns){ //last node set of first row
		if(nodesInGraph[aNode]==UNLOCKED){
			switch(aNode%4){
				case 0: nodeSet[0]=aNode+1; nodeSet[1]=aNode+3; return 2;
				case 1: nodeSet[0]=aNode+1; nodeSet[1]=aNode-1; nodeSet[2]=aNode+columns; return 3;
				case 2: nodeSet[0]=aNode+1; nodeSet[1]=aNode-1; nodeSet[2]=aNode-4; return 3;
				case 3: nodeSet[0]=aNode-3; nodeSet[1]=aNode-1; return 2;
			}
		}
		else{
			switch(aNode%4){
				case 0: return 0;
				case 1: nodeSet[0]=aNode+columns; return 1;
				case 2: nodeSet[0]=aNode-4; return 1;
				case 3: return 0;
			}
		}
	}
	//check
	else if(aNode<columns*(rows-1)){ //set of intermediate rows
		if(aNode%columns<4){ //left side node set of intermediate rows
			if(nodesInGraph[aNode]==UNLOCKED){
				switch(aNode%4){
					case 0: nodeSet[0]=aNode+1; nodeSet[1]=aNode+3; nodeSet[2]=aNode+4; return 3;
					case 1: nodeSet[0]=aNode+1; nodeSet[1]=aNode-1; nodeSet[2]=aNode+columns; return 3;
					case 2: nodeSet[0]=aNode+1; nodeSet[1]=aNode-1; return 2;
					case 3: nodeSet[0]=aNode-3; nodeSet[1]=aNode-1; nodeSet[2]=aNode-columns; return 3;
				}
			}
			else{
				switch(aNode%4){
					case 0: nodeSet[0]=aNode+4; return 1;
					case 1: nodeSet[0]=aNode+columns; return 1;
					case 2: return 0;
					case 3: nodeSet[0]=aNode-columns; return 1;
				}
			}
		}
		else if(aNode%columns<(columns-4)){ //intermediate node set of intermediate rows
			if(nodesInGraph[aNode]==UNLOCKED){
				switch(aNode%4){
					case 0: nodeSet[0]=aNode+1; nodeSet[1]=aNode+3; nodeSet[2]=aNode+4; return 3;
					case 1: nodeSet[0]=aNode+1; nodeSet[1]=aNode-1; nodeSet[2]=aNode+columns; return 3;
					case 2: nodeSet[0]=aNode+1; nodeSet[1]=aNode-1; nodeSet[2]=aNode-4; return 3;
					case 3: nodeSet[0]=aNode-3; nodeSet[1]=aNode-1; nodeSet[2]=aNode-columns; return 3;
				}
			}
			else{
				switch(aNode%4){
					case 0: nodeSet[0]=aNode+4; return 1;
					case 1: nodeSet[0]=aNode+columns; return 1;
					case 2: nodeSet[0]=aNode-4; return 1;
					case 3: nodeSet[0]=aNode-columns; return 1;
				}
			}
		}
		else{ //right side node set of intermediate rows
			if(nodesInGraph[aNode]==UNLOCKED){
				switch(aNode%4){
					case 0: nodeSet[0]=aNode+1; nodeSet[1]=aNode+3; return 2;
					case 1: nodeSet[0]=aNode+1; nodeSet[1]=aNode-1; nodeSet[2]=aNode+columns; return 3;
					case 2: nodeSet[0]=aNode+1; nodeSet[1]=aNode-1; nodeSet[2]=aNode-4; return 3;
					case 3: nodeSet[0]=aNode-3; nodeSet[1]=aNode-1; nodeSet[2]=aNode-columns; return 3;
				}
			}
			else{
				switch(aNode%4){
					case 0: return 0;
					case 1: nodeSet[0]=aNode+columns; return 1;
					case 2: nodeSet[0]=aNode-4; return 1;
					case 3: nodeSet[0]=aNode-columns; return 1;
				}
			}
		}
	}
	else if(aNode-(columns*(rows-1))<4){ //first node set of last row
		if(nodesInGraph[aNode]==UNLOCKED){
			switch(aNode%4){
				case 0: nodeSet[0]=aNode+1; nodeSet[1]=aNode+3; nodeSet[2]=aNode+4; return 3;
				case 1: nodeSet[0]=aNode+1; nodeSet[1]=aNode-1; return 2;
				case 2: nodeSet[0]=aNode+1; nodeSet[1]=aNode-1; return 2;
				case 3: nodeSet[0]=aNode-3; nodeSet[1]=aNode-1; nodeSet[2]=aNode-columns; return 3;
			}
		}
		else{
			switch(aNode%4){
				case 0: nodeSet[0]=aNode+4; return 1;
				case 1: return 0;
				case 2: return 0;
				case 3: nodeSet[0]=aNode-columns; return 1;
			}
		}
	}
	else if(aNode-(columns*(rows-1))<(columns-4)){ //intermediate node set of last row
		if(nodesInGraph[aNode]==UNLOCKED){
			switch(aNode%4){
				case 0: nodeSet[0]=aNode+1; nodeSet[1]=aNode+3; nodeSet[2]=aNode+4; return 3;
				case 1: nodeSet[0]=aNode+1; nodeSet[1]=aNode-1; return 2;
				case 2: nodeSet[0]=aNode+1; nodeSet[1]=aNode-1; nodeSet[2]=aNode-4; return 3;
				case 3: nodeSet[0]=aNode-3; nodeSet[1]=aNode-1; nodeSet[2]=aNode-columns; return 3;
			}
		}
		else{
			switch(aNode%4){
				case 0: nodeSet[0]=aNode+4; return 1;
				case 1: return 0;
				case 2: nodeSet[0]=aNode-4; return 1;
				case 3: nodeSet[0]=aNode-columns; return 1;
			}
		}
	}
	else{ //last node set of last row
		if(nodesInGraph[aNode]==UNLOCKED){
			switch(aNode%4){
				case 0: nodeSet[0]=aNode+1; nodeSet[1]=aNode+3; return 2;
				case 1: nodeSet[0]=aNode+1; nodeSet[1]=aNode-1; return 2;
				case 2: nodeSet[0]=aNode+1; nodeSet[1]=aNode-1; nodeSet[2]=aNode-4; return 3;
				case 3: nodeSet[0]=aNode-3; nodeSet[1]=aNode-1; nodeSet[2]=aNode-columns; return 3;
			}
		}
		else{
			switch(aNode%4){
				case 0: return 0;
				case 1: return 0;
				case 2: nodeSet[0]=aNode-4; return 1;
				case 3: nodeSet[0]=aNode-columns; return 1;
			}
		}
	}
}

void Dijkstra::relax(unsigned int u, unsigned int v){
	int cost = getWeight(u,v);
	if(distance[v]>distance[u]+cost){
		distance[v] = distance[u]+cost;
		predecessor[v]  = u;
		if((u-u%4)==(v-v%4))
			nodeLevel[v] = nodeLevel[u];
		else
			nodeLevel[v] = nodeLevel[u]+1;
	}
}

bool Dijkstra::isGoalNode(unsigned int currrentNode, unsigned int goalInSimpleCoordinates){
	if(currrentNode/4==goalInSimpleCoordinates)
		return true;
	else
		return false;
}

unsigned int Dijkstra::getWeight(unsigned int u, unsigned int v){
	
	if(u/4==v/4){ //sub nodes
		int debug = turningAngles[u%4][v%4];
		return TURNING_COST*abs((int)turningAngles[u%4][v%4]);
	}
	else{
		return NODE_TRAVEL_COST;
	}	
}

bool Dijkstra::getShortestPath(unsigned int startNodeInSimpleCoordinate, unsigned int currentDirection, unsigned int goalNodeInSimpleCoordinate, path *optimalPath){
	
	unsigned int outDegree[OUT_DEGREE_MAX];
	
	initialileSingleSource(startNodeInSimpleCoordinate*4+currentDirection); //startNode = startNodeInSimpleCoordinate*4+currentDirection
	for(int iteration=0; iteration<NO_OF_NODES; iteration++){
		
		unsigned int currentNode = extractMinNonVisited();
		nodeState[currentNode] = VISITED;
		
		if(isGoalNode(currentNode, goalNodeInSimpleCoordinate)){ //search is finished

			if(distance[currentNode]==NOT_REACHABLE) //no path
				return false;
			//construct the path
			optimalPath->length = nodeLevel[currentNode];
			unsigned int currentPathNode = currentNode;
			unsigned int currentLevel = nodeLevel[currentNode]+1;
			while(nodeLevel[currentPathNode]!=0){
				
				if(nodeLevel[currentPathNode] != currentLevel){ //new main Node
					optimalPath->nodes[--currentLevel-1] = createNode((currentPathNode/4)/(NO_OF_GRID_COLUMNS-1), (currentPathNode/4)%(NO_OF_GRID_COLUMNS-1));
					//currentLevel--;
				}
				currentPathNode = predecessor[currentPathNode];
			}
			return true;
		}
		
		int noOfOutDegree = getNumberOfOutgoingConnections(currentNode, outDegree);
		for(int iteration=0; iteration<noOfOutDegree; iteration++){
			if(nodesInGraph[outDegree[iteration]]==BLOCKED){
				if(isGoalNode(outDegree[iteration], goalNodeInSimpleCoordinate))
					relax(currentNode, outDegree[iteration]);
				else
					continue;
			}
			else
				relax(currentNode, outDegree[iteration]);
		}
	}
	return false;
}

bool Dijkstra::getShortestPath(node startNode, unsigned int currentDirection, node goalNode, path *optimalPath){
	
	unsigned int startNodeInSimpleCoordinate = startNode.x*(NO_OF_GRID_COLUMNS-1)+startNode.y;
	unsigned int goalNodeInSimpleCoordinate = goalNode.x*(NO_OF_GRID_COLUMNS-1)+goalNode.y;
	
	unsigned int outDegree[OUT_DEGREE_MAX];
	
	initialileSingleSource(startNodeInSimpleCoordinate*4+currentDirection); //startNode = startNodeInSimpleCoordinate*4+currentDirection
	for(int iteration=0; iteration<NO_OF_NODES; iteration++){
		
		unsigned int currentNode = extractMinNonVisited();
		nodeState[currentNode] = VISITED;
		
		if(isGoalNode(currentNode, goalNodeInSimpleCoordinate)){ //search is finished

			if(distance[currentNode]==NOT_REACHABLE) //no path
				return false;

			//construct the path
			optimalPath->length = nodeLevel[currentNode];
			unsigned int currentPathNode = currentNode;
			unsigned int currentLevel = nodeLevel[currentNode]+1;
			while(nodeLevel[currentPathNode]!=0){
				
				if(nodeLevel[currentPathNode] != currentLevel){ //new main Node
					optimalPath->nodes[--currentLevel-1] = createNode((currentPathNode/4)/(NO_OF_GRID_COLUMNS-1), (currentPathNode/4)%(NO_OF_GRID_COLUMNS-1));
					//currentLevel--;
				}
				currentPathNode = predecessor[currentPathNode];
			}
			return true;
		}
		
		int noOfOutDegree = getNumberOfOutgoingConnections(currentNode, outDegree);
		for(int iteration=0; iteration<noOfOutDegree; iteration++){
			if(nodesInGraph[outDegree[iteration]]==BLOCKED){
				if(isGoalNode(outDegree[iteration], goalNodeInSimpleCoordinate))
					relax(currentNode, outDegree[iteration]);
				else
					continue;
			}
			else
				relax(currentNode, outDegree[iteration]);
		}
	}
	return false;
}

void Dijkstra::printNodesStatus(){
	Serial.println();
	for(int i=0; i<(NO_OF_GRID_ROWS-1); i++){
		for(int j=0; j<(NO_OF_GRID_COLUMNS-1)*4; j++){
			Serial.print(nodesInGraph[i*(NO_OF_GRID_COLUMNS-1)*4+j]);
			Serial.print(" ");
		}
		Serial.println();
	}
	Serial.println();
}

void Dijkstra::printStatus(){
	Serial.println();
	Serial.println("Distance:");
	for(int i=0; i<(NO_OF_GRID_ROWS-1); i++){
		for(int j=0; j<(NO_OF_GRID_COLUMNS-1)*4; j++){
			Serial.print(distance[i*(NO_OF_GRID_COLUMNS-1)*4+j]);
			Serial.print(" ");
		}
		Serial.println();
	}
	Serial.println();
	
	Serial.println();
	Serial.println("Predecessor:");
	for(int i=0; i<(NO_OF_GRID_ROWS-1); i++){
		for(int j=0; j<(NO_OF_GRID_COLUMNS-1)*4; j++){
			Serial.print(predecessor[i*(NO_OF_GRID_COLUMNS-1)*4+j]);
			Serial.print(" ");
		}
		Serial.println();
	}
	Serial.println();
	
	Serial.println();
	Serial.println("Visited:");
	for(int i=0; i<(NO_OF_GRID_ROWS-1); i++){
		for(int j=0; j<(NO_OF_GRID_COLUMNS-1)*4; j++){
			Serial.print(nodeState[i*(NO_OF_GRID_COLUMNS-1)*4+j]);
			Serial.print(" ");
		}
		Serial.println();
	}
	Serial.println();
	
	Serial.println();
	Serial.println("Level:");
	for(int i=0; i<(NO_OF_GRID_ROWS-1); i++){
		for(int j=0; j<(NO_OF_GRID_COLUMNS-1)*4; j++){
			Serial.print(nodeLevel[i*(NO_OF_GRID_COLUMNS-1)*4+j]);
			Serial.print(" ");
		}
		Serial.println();
	}
	Serial.println();
}


#endif