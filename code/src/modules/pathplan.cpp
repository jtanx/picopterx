/**
 * @file pathplan.cpp
 * @brief Obstacle avoidance.
 */

#include "common.h"
#include "pathplan.h"
#include <fstream>

using namespace picopter;
using namespace picopter::navigation;

PathPlan::PathPlan(){
    numNodes = 0;
    edgeMatrixSize =64;
    
    //set up edge matrices
    collisionBoundary.resize(edgeMatrixSize);
    paths.resize(edgeMatrixSize);
    for(int i = 0; i<edgeMatrixSize; i++){
        collisionBoundary[i].resize(edgeMatrixSize);
        paths[i].resize(edgeMatrixSize);
    } 
    
    for(int i = 0; i<edgeMatrixSize; i++){
        for(int j = 0; i<edgeMatrixSize; i++) collisionBoundary[i][j] = -1.0;
        for(int j = 0; i<edgeMatrixSize; i++) paths[i][j] = -1.0;
    }
}

void PathPlan::addPolygon(std::deque<Coord3D> c){
	polygonSides.push_back(c.size());
    
    int startpoint = numNodes;
    
    for (size_t i=0; i<c.size(); i++){
        addNode(c[i].lat, c[i].lon);
        
        //join node to previous node with edge
        if(numNodes-1 > startpoint) addCollisionEdge(numNodes-2, numNodes-1);
    }
    //close the polygon by creating an edge 'twixt first and last nodes
    addCollisionEdge(numNodes-1, startpoint);
    
    //Generate traversable paths with the rest of the graph
	for(int i =startpoint; i < numNodes; i++){      // for each node of the new polgon
	    for(int j = 0; j < startpoint; j++){        // with every other node in the graph
	    	bool traversable = true;
	        
	        for(int k = 0; k < numNodes; k++){      //compare with the edges of every other polygon for intersections
	            for(int l = k; l < numNodes; l++){
	                if(collisionBoundary[k][l] > 0){
	                    if( checkIntersection(i, j, k, l) ){ 
	                        traversable = false;
	                        break;
	                    }
	                }
	            }
	        }
	        if(traversable) {                       //if there are no intersections with collision zones, create a traversable path
	            addPathEdge(i, j);
	        }
	    }
	}
}

//takes a deque of waypoints and returns a new deque of waypoints that avoids collision zones
std::deque<Waypoints::Waypoint> PathPlan::generateFlightPlan(std::deque<Waypoints::Waypoint> waypoints){
    std::deque<Waypoints::Waypoint> flightPlan;


    while(waypoints.size() > 1){
        std::deque<int> path = detour(waypoints[0].pt, waypoints[1].pt);
        
        for(size_t i =0; i < path.size()-1; i++){
            Waypoints::Waypoint waypoint = waypoints[0]; //TODO check for validity
            waypoint.pt.lon = nodes[path[i]].x;
            waypoint.pt.lat = nodes[path[i]].y;
            flightPlan.push_back(waypoint);
        }
        
        waypoints.pop_front();
    }
    flightPlan.push_back(waypoints[0]);
    waypoints.pop_front();
    
//for(int i = 0; i< flightPlan.size(); i++) std::cout << flightPlan[i].lat << ", " << flightPlan[i].lon << "\n";

    return flightPlan;
}


void PathPlan::addNode(double lat, double lon){
    //if(numNodes > edgeMatrixSize) resizeEdgeMatrices();
    
    node n;
    n.y = lat;
    n.x = lon;
    nodes.push_back(n);  
    
    numNodes++; 
}

void PathPlan::deleteNode(int index){
    nodes.erase (nodes.begin()+index);
    //remove from edge martices' row
    for(int i=0; i<numNodes; i++){
        paths[i].erase (paths[i].begin()+index);
        collisionBoundary[i].erase (collisionBoundary[i].begin()+index);
    }
    //remove edge matrices' column
    paths.erase (paths.begin()+index);
    collisionBoundary.erase (collisionBoundary.begin()+index);
    
    numNodes--;
}

//add an edge demarcating a collision zone
void PathPlan::addCollisionEdge(int n1, int n2){
    double length = sqrt(
             	pow(nodes[n2].x - nodes[n1].x, 2) +
             	pow(nodes[n2].y - nodes[n1].y, 2)
             );
    collisionBoundary[n1][n2] = length;
    collisionBoundary[n2][n1] = length;
    
    addPathEdge(n1, n2);
}

//add an edge describing a traversable path
void PathPlan::addPathEdge(int n1, int n2){
    double length = sqrt(
             	pow(nodes[n2].x - nodes[n1].x, 2) +
             	pow(nodes[n2].y - nodes[n1].y, 2)
             );
    paths[n1][n2] = length;
    paths[n2][n1] = length;
    
}


//get the cross product of two 2d vectors
double PathPlan::crossProduct(double Ax, double Ay, double Bx, double By){
    return Ax*By - Ay*Bx;
}

//check if two line-segments cross
bool PathPlan::checkIntersection(int node1, int node2, int node3, int node4){
	node n1=nodes[node1], n2=nodes[node2], n3=nodes[node3], n4=nodes[node4];
    //if no line exists to intersect, no intersection
    if(collisionBoundary[node3][node4] < 0) return false;

	//check if endpoints of line 2 lie on opposite sides of line 1, and vice versa.
    // How: taking the cross-product of a line's direction vector and point vector will yeild a positive scalar if the point is on one side of the line, and a negative scapar if the point lies on the other side of the line.
    //Therefore, if we multiply the results for both endpoints, the result will be positive if the two points are on the same side, and negative if they are on opposite sides.
    //We can test for intersection by asking if(result) <0.
    //Since we are dealing with line segments, we must repeat the test with the other line to cover all cases.   
    bool intersects1 =
    crossProduct(	n2.x-n1.x, n2.y-n1.y,
                	n3.x-n1.x, n3.y-n1.y )*
    crossProduct(   n2.x-n1.x, n2.y-n1.y,
                	n4.x-n1.x, n4.y-n1.y ) < 0.0;
                	
    bool intersects2 =
    crossProduct(  	n4.x-n3.x, n4.y-n3.y,
                    n1.x-n3.x, n1.y-n3.y )*
    crossProduct(   n4.x-n3.x, n4.y-n3.y,
                    n2.x-n3.x, n2.y-n3.y ) < 0.0;

    return intersects1 && intersects2;
}

double PathPlan::displacement(node n1, node n2){
    return sqrt(
                pow(n2.x - n1.x, 2) +
             	pow(n2.y - n1.y, 2)
           );
}


//generate shortest path between two coordinates that avoids collision zones
std::deque<int> PathPlan::detour(Coord3D A, Coord3D B ){
	int start = numNodes; //index of the path origin
	addNode(A.lat, A.lon);
	
	int end = numNodes; //index of the path terminus
	addNode(B.lat, B.lon);
	
	//add new start and end nodes to graph, generate traversable paths to nodes
	for(int i =0; i<numNodes; i++){
		bool traversableFromStart = true;
		bool traversableToEnd = true;
		for(int j =0; j<numNodes; j++){
		    for(int k = j; k<numNodes; k++){
			    if( checkIntersection(start, i, j, k) ) traversableFromStart=false;
			    if( checkIntersection(end, i, j, k) )  traversableToEnd=false;
			    if(!traversableFromStart && !traversableToEnd) break;
			}
		}
		if(traversableFromStart) addPathEdge(start, i);
		if(traversableToEnd) addPathEdge(i,end);
	}
	
	//use A* algorithm to find shortest path from start to end
	
	std::vector<double> distance(numNodes);
	std::vector<double>  fitness(numNodes);
	std::vector<int> pathtree(numNodes);	
	std::deque<int> OPEN;
	std::deque<int> CLOSED;
	
	OPEN.push_front(start);
	fitness[start] = displacement(nodes[start], nodes[end]);
	
	//A* main loop
	bool success = false;
	while(OPEN.size() > 0 && !success){
		int n = 0; //index of OPEN with most-fit node
		//find most-fit discovered node
		for(size_t i = 1; i<OPEN.size();i++){
		    if(fitness[OPEN[i]]<fitness[OPEN[n]]) n=i;
		}
std::cout << OPEN[n] << ": ";
		//check if node is destination
		if(OPEN[n] == end){
		    success = true;
std::cout << "success!\n";
		    break;
		}
		
		//enqueue all of most-fit node's neighbors
		for(int i = 0; i< numNodes; i++){
		    bool explored = false;
		    for(size_t j= 0; j<OPEN.size(); j++) if(OPEN[j]==i) explored = true;
		    for(size_t j= 0; j<CLOSED.size(); j++) if(CLOSED[j]==i) explored = true;
		    		    
		    if(!explored && paths[OPEN[n]][i] > 0){
std::cout << i << ", ";
			    distance[i] = distance[OPEN[n]]+paths[OPEN[n]][i];
			    fitness[i] =distance[i] + displacement(nodes[OPEN[n]], nodes[i]);
		         pathtree[i] = OPEN[n];
		         OPEN.push_front(i);
		    }
		}
		CLOSED.push_front(OPEN[n]);
		OPEN.erase (OPEN.begin()+n);
std::cout << "\n";
	}

    deleteNode(end);
    deleteNode(start);	
    
for(int i=0; i<numNodes; i++) std::cout << pathtree[i] << ", ";
    
    //create path from pathtree
    std::deque<int> path;
    int backstep = end;
    while(backstep != start){
std::cout << backstep << "\n";
        path.push_front(backstep);
        backstep = pathtree[backstep];
    }
    path.push_front(start);
    
    return path;
}

//Writes graph to an .svg file
void PathPlan::writeGraphSVGJamesOval(const char *fileName, std::deque<Waypoints::Waypoint> flightPlan){
    
    double originy = -31.979272, originx = 115.817147;
    double terminy = -31.980967, terminx = 115.818789;
    double width = 417, height = 505;
	
	//write header
	std::ofstream map;
    map.open (fileName);
    map << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n\n<svg\n    id=\"svg6165\"\n    version=\"1.1\"\n    width=\"417\"\n    height=\"505\">\n\n";
    map << "    <image\n     width=\"417\"  \n     height=\"505\" \n     xlink:href=\"file:///home/wash/Documents/workspace/Screenshot%20from%202015-09-13%2017:02:44.png\"  \n     id=\"image6173\"\n     x=\"0\"\n     y=\"0\" />\n";
    
    //Draw collision zones
    int index = 0;
    for(size_t i = 0; i<polygonSides.size(); i++){
        map << "   <path\n     style=\"fill:#ff0000;stroke:none;stroke-width:1px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1;fill-opacity:1;opacity:0.5\"\n" << "        d=\"M   ";
 
        for(int j=0; j<polygonSides[i]; j++){
            map << (nodes[index].x-originx)/(terminx-originx) * width  << ',' << (nodes[index].y-originy)/(terminy-originy) * height << " ";
            index++;
        }
        map << " Z\"\n     id=\"path3068\"/>\n";
    }
    
    //Draw traversable paths
    for(int i=0; i<numNodes-1; i++){
        for(int j=i+1; j<numNodes; j++){
            if(paths[i][j] > 0){
                map << "   <path\n     style=\"fill:none;stroke:#000000;stroke-width:1px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1\"\n     d=\"M ";
                map << (nodes[i].x-originx)/(terminx-originx) * width  << ',' << (nodes[i].y-originy)/(terminy-originy) * height;
                map << ' ';
                map << (nodes[j].x-originx)/(terminx-originx) * width  << ',' << (nodes[j].y-originy)/(terminy-originy) * height;
                map << "\"\n     id=\"path" << i << j << "\"/>\n";
            }
        }
    }
    
    //Draw flight plan
    if(flightPlan.size() > 1){
        map << "   <path\n     style=\"fill:none;stroke:#ffff00;stroke-width:3px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1\"\n     d=\"M ";
        for(size_t i = 0; i<flightPlan.size(); i++){
            map << (flightPlan[i].pt.lon - originx)/(terminx-originx) * width  << ',' << (flightPlan[i].pt.lat-originy)/(terminy-originy) * height;
            map << ' ';
        }
        map << "\"\n     id=\"path" << " flightplan" << "\"/>\n";
    }
    
    map << "</svg>\n";
    
    map.close();
    
}
