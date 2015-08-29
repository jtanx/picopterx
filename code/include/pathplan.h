/**
 * @file pathplan.h
 * @brief Obstacle avoidance path planner.
 **/

#ifndef _PICOPTERX_PATH_PLAN_H
#define _PICOPTERX_PATH_PLAN_H

/* For the Options class */
#include "opts.h"
#include "waypoints.h"
/* OGDF stuff */
#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/fileformats/GraphIO.h>
#include <ogdf/basic/Graph_d.h>

namespace picopter {
    class PathPlan {
        public:
            typedef struct CollisionZone {
	            std::deque<navigation::Coord3D> polygon;
            } CollisionZone;
            
		    PathPlan();
		    void addPolygon(CollisionZone c);
		    void writeGraphSVG(const char *fileName);
		    std::deque<Waypoints::Waypoint> detour(std::deque<Waypoints::Waypoint> waypoints);
	    private:
		    ogdf::Graph graph;
		    ogdf::GraphAttributes GA;
		    ogdf::NodeArray<ogdf::node> nodes;
		    int numNodes;
		    ogdf::EdgeArray<ogdf::edge> collisionBoundary;
		    ogdf::EdgeArray<ogdf::edge> paths;
    };
}

#endif // _PICOPTERX_PATH_PLAN_H

