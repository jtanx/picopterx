/**
 * @file pathplan.cpp
 * @brief Obstacle avoidance.
 */

#include "common.h"
#include "pathplan.h"

using namespace ogdf;
using namespace picopter;
using namespace picopter::navigation;

PathPlan::PathPlan(void)
: GA(graph, GraphAttributes::nodeGraphics | GraphAttributes::edgeGraphics)
, nodes(graph)
, collisionBoundary(graph)
, paths(graph)
{
	numNodes = 0;
}

void PathPlan::addPolygon(CollisionZone c){
	int startpoint = numNodes;
	int index = numNodes;
	for (size_t i=0; i<c.polygon.size(); i++){
	
		node thisNode = graph.newNode();
		GA.x(thisNode) = c.polygon[i].lat;
		GA.y(thisNode) = c.polygon[i].lon;
		GA.width(thisNode) = 4;
		GA.height(thisNode) = 4;
		nodes[index] = thisNode;
		
		//join node to previous node with edge
		if(index > startpoint) collisionBoundary[index-1] = graph.newEdge(nodes[index], nodes[(index-1)] );
		index++;
	}
	//close the polygon by creating an edge 'twixt first and last nodes
	collisionBoundary[index-1] = graph.newEdge(nodes[index-1], nodes[startpoint]);
}

//writes graph to SVG file
void PathPlan::writeGraphSVG(const char *fileName){
	GraphIO::SVGSettings settings;
	settings.margin(20.0);
 	GraphIO::drawSVG(GA, fileName, settings);
}

std::deque<Waypoints::Waypoint> PathPlan::detour(std::deque<Waypoints::Waypoint> waypoints){
	return waypoints;
}

