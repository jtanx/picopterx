/**
 * @file pathplan.h
 * @brief Obstacle avoidance path planner.
 **/

#ifndef _PICOPTERX_PATH_PLAN_H
#define _PICOPTERX_PATH_PLAN_H

/* For the Options class */
#include "opts.h"
#include "waypoints.h"

namespace picopter {
    class PathPlan{
        
        public:
            typedef struct node {
	            double x;
	            double y;
            } node;
            
	        PathPlan();
	        void addPolygon(std::deque<navigation::Coord3D> c);
	        std::deque<Waypoints::Waypoint> generateFlightPlan(std::deque<Waypoints::Waypoint> waypoints);
	        void writeGraphSVGJamesOval(const char *fileName, std::deque<navigation::Coord3D> flightPlan);
        private:
            int numNodes;
	        std::vector<node> nodes;
	        std::vector< std::vector<double> > collisionBoundary; //stores obstacle edges
	        std::vector< std::vector<double> > paths; //stores paths
	        int edgeMatrixSize;
	        std::vector<int> polygonSides;
	        
	        void addNode(double lat, double lon);
	        void deleteNode(int index);    
	        void addCollisionEdge(int n1, int n2);
	        void addPathEdge(int n1, int n2);
	        double crossProduct(double Ax, double Ay, double Bx, double By);
	        double displacement(node n1, node n2);
		    bool checkIntersection(int node1, int node2, int node3, int node4);
		    std::deque<int> detour(navigation::Coord3D A, navigation::Coord3D B);
    };
}

#endif // _PICOPTERX_PATH_PLAN_H

