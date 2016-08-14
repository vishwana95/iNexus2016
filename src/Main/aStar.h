// Authors: Diunuge Buddhika Wijesinghe
// Competition iNexus 2013
// version: 2.0v: Not Optimized
// Last mod: 27-11-2012


#ifndef Diu_A_Star
#define Diu_A_Star
	
#include <wprogram.h>
#include <math.h>
#include "Lib\stlastar.h"

//using namespace std;

// The world map

#define  MAP_WIDTH 8
#define  MAP_HEIGHT 7

int gridMap[ MAP_WIDTH * MAP_HEIGHT ] = 
{
//  0 1 2 3 4 5 6 7
	1,1,1,1,1,1,1,1,   // 0
	1,9,9,1,9,9,9,9,   // 1
	1,9,9,1,1,9,9,9,   // 2
	1,9,9,1,1,9,9,9,   // 3
	1,9,1,1,1,1,9,9,   // 4
	1,9,1,1,9,1,1,1,   // 5
	1,9,9,9,9,1,1,1,   // 6
};

// map helper functions

int GetMap( int x, int y )
{
	if( x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT ) //Out of bound
		return 9;	 

	return gridMap[(y*MAP_WIDTH)+x];
}

// Definitions

class MapSearchNode
{
public:
	unsigned int x;	 // the (x,y) positions of the node
	unsigned int y;	
	
	MapSearchNode() { x = y = 0; }
	MapSearchNode( unsigned int px, unsigned int py ) { x=px; y=py; }

	float GoalDistanceEstimate( MapSearchNode &nodeGoal );
	bool IsGoal( MapSearchNode &nodeGoal );
	bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
	float GetCost( MapSearchNode &successor );
	bool IsSameState( MapSearchNode &node );

	void PrintNodeInfo(); 

};

bool MapSearchNode::IsSameState( MapSearchNode &node )
{
	// same state in a maze search is simply when (x,y) are the same
	if( (x == node.x) && (y == node.y) )
		return true;
	else
		return false;
}

void MapSearchNode::PrintNodeInfo()
{
	Serial.println();
	//cout << "Node : (" << x << ", " << y << ")" << endl;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal. 

float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
	float xd = fabs(float(((float)x - (float)nodeGoal.x)));
	float yd = fabs(float(((float)y - (float)nodeGoal.y)));

	return xd + yd;
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{
	return IsSameState(nodeGoal);
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{

	int parent_x = -1; 
	int parent_y = -1; 

	if( parent_node )
	{
		parent_x = parent_node->x;
		parent_y = parent_node->y;
	}
	

	MapSearchNode NewNode;

	// push each possible move except allowing the search to go backwards

	if( (GetMap( x-1, y ) < 9)  && !((parent_x == x-1) && (parent_y == y)) ) 
	{
		NewNode = MapSearchNode( x-1, y );
		astarsearch->AddSuccessor( NewNode );
	}	

	if( (GetMap( x, y-1 ) < 9)  && !((parent_x == x) && (parent_y == y-1)) ) 
	{
		NewNode = MapSearchNode( x, y-1 );
		astarsearch->AddSuccessor( NewNode );
	}	

	if( (GetMap( x+1, y ) < 9) && !((parent_x == x+1) && (parent_y == y)) ) 
	{
		NewNode = MapSearchNode( x+1, y );
		astarsearch->AddSuccessor( NewNode );
	}	

		
	if( (GetMap( x, y+1 ) < 9)  && !((parent_x == x) && (parent_y == y+1)) )
	{
		NewNode = MapSearchNode( x, y+1 );
		astarsearch->AddSuccessor( NewNode );
	}	

	return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving

float MapSearchNode::GetCost( MapSearchNode &successor )
{
	return (float) GetMap( x, y );
}

#endif 