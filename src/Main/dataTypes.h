// Authors: Diunuge Buddhika Wijesinghe
// Competition iNexus 2013
// version: 2.0v: Not Optimized
// Last mod: 11-11-2012


#ifndef diuDataTypes_H
#define diuDataTypes_H


/*!
 * @typedef     node
 *              Represent node of the grid.
 * @field       x     reference x coordinate.
 * @field       y     reference y coordinate.
*/
typedef struct _node{
	int x;
	int y;
}node;


/*!
 * @typedef     path
 *              Represent a path by list of arranged nodes.
 * @field       length     number of nodes included in the path.
 * @field       nodes      array of nodes.
*/
typedef struct _path{
	int length;
	node* nodes;
}path,shortestPath;


class MapNode
{
public:
	unsigned int x;	 // the (x,y) positions of the node
	unsigned int y;	
	
	MapNode() { x = y = 0; }
	MapNode( unsigned int px, unsigned int py ) { x=px; y=py; }

};


/*!
 * @function    createNode
 * @abstract    Create node
 * @discussion  This creates node variable using coordinates.
 * @param       x    x coordinate
 * @param       y    y coordinate
 * @result      returns a cunstructed node.
*/
node createNode(int x, int y){
	node aNode = {x,y};
	return aNode;
}

#endif