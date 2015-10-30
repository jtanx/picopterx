/**
 * @file pathplan.cpp
 * @brief Obstacle avoidance.
 */

#include "common.h"
#include "pathplan.h"
#include <fstream>

using namespace picopter;
using namespace picopter::navigation;

/** 
 * Constructor. Constructs with default settings.
 */
PathPlan::PathPlan(GridSpace *g){
    numNodes = 0;
    //errorRadius = 2*asin(6.0 / (2*6378137.00) );//6 metres
    errorRadius = 0.00003;
    
    //set up edge matrices
    /*
    edgeMatrixSize =64;
    collisionBoundary.resize(edgeMatrixSize);
    paths.resize(edgeMatrixSize);
    for(int i = 0; i<edgeMatrixSize; i++){
        collisionBoundary[i].resize(edgeMatrixSize);
        paths[i].resize(edgeMatrixSize);
    } 
    for(int i = 0; i<edgeMatrixSize; i++){
        for(int j = 0; j<edgeMatrixSize; j++){
            collisionBoundary[i][j] = -1.0;
            paths[i][j] = -1.0;
        }
    }
    */
    //point to gridspace world, create polygons around obstacles
    gridSpace = g;
    readGridSpace();
}

/** 
 * Adds a polygon to the collision zone graph.
 * @param [in] c a std::deque of coordinates describing the points of the polygon.
 */
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
}

/**
 * Takes a deque of waypoints and returns a new deque of waypoints that avoids collision zones
 * @param [in] waypoints A std::deque of waypoints.
 * @return A std::deque of waypoints.
 */
std::deque<Waypoints::Waypoint> PathPlan::generateFlightPlan(std::deque<Waypoints::Waypoint> waypoints){
    std::deque<Waypoints::Waypoint> flightPlan;
    generateGraph();  
    if(fencePosts.size() == 0) return waypoints;
     
    while(waypoints.size() > 1){
        std::deque<int> path = detour(waypoints[0].pt, waypoints[1].pt);
   
        for(size_t i =0; i < path.size()-1; i++){
            Waypoints::Waypoint waypoint = waypoints[0]; 
            waypoint.pt.lon = fencePosts[path[i]].x;
            waypoint.pt.lat = fencePosts[path[i]].y;
            flightPlan.push_back(waypoint);
       }
        
        waypoints.pop_front();
    }
    flightPlan.push_back(waypoints[0]);
    waypoints.pop_front();
    
    return flightPlan;
}

/**
 * Adds a node to the collision graph.
 * @param [in] lat the latitude of the node's location.
 * @param [in] lon the longitude of the node's location.
 */
void PathPlan::addNode(double lat, double lon){
    //if(nodes.size() > edgeMatrixSize) resizeEdgeMatrices();
    
    node n;
    n.y = lat;
    n.x = lon;
    nodes.push_back(n); 
    
    std::vector <double> edges;
    edges.resize(nodes.size());
    for(size_t i=0; i<nodes.size(); i++) edges[i] = -1;
    collisionBoundary.push_back(edges); 
    for(size_t i=0; i<nodes.size()-1; i++) collisionBoundary[i].push_back( -1.0);
    
    numNodes++; 
}

/**
 * Adds a node to the traversable graph.
 * @param [in] lat the latitude of the node's location.
 * @param [in] lon the longitude of the node's location.
 */
void PathPlan::addFencePost(double lon, double lat){
    
    node n;
    n.y = lat;
    n.x = lon;
    fencePosts.push_back(n); 
    
    std::vector <double> edges;
    edges.resize(fencePosts.size());
    for(size_t i=0; i<fencePosts.size(); i++) edges[i] = -1;
    paths.push_back(edges); 
    for(size_t i=0; i<fencePosts.size()-1; i++) paths[i].push_back( -1.0); 
    
}

/**
 * Removes a node from the collision graph.
 * @param [in] index The index of the node.
 */
void PathPlan::deleteNode(int index){
    nodes.erase (nodes.begin()+index);
    //remove from edge martices' row
    for(size_t i=0; i<nodes.size(); i++){
        collisionBoundary[i].erase (collisionBoundary[i].begin()+index);
    }
    //remove edge matrices' column
    collisionBoundary.erase (collisionBoundary.begin()+index);
    
    numNodes--;
}

/**
 * Removes a node from the traversable graph.
 * @param [in] index The index of the node.
 */
void PathPlan::deleteFencePost(int index){

    //remove from edge martices' row
    for(size_t i=0; i<fencePosts.size(); i++){
        paths[i].erase (paths[i].begin()+index);
    }
    //remove edge matrices' column
    paths.erase (paths.begin()+index);
    
    //remove the node
    fencePosts.erase (fencePosts.begin()+index);
}

/**
 * Add an edge demarcating a collision zone.
 * @param [in] n1 The index of the origin node.
 * @param [in] n2 The index of the terminating node.
 */
void PathPlan::addCollisionEdge(int n1, int n2){
    double length = sqrt(
             	pow(nodes[n2].x - nodes[n1].x, 2) +
             	pow(nodes[n2].y - nodes[n1].y, 2)
             );
    collisionBoundary[n1][n2] = length;
    collisionBoundary[n2][n1] = length;

}

/**
 * Add an edge describing a traversable path.
 * @param [in] n1 The index of the origin node.
 * @param [in] n2 The index of the terminating node.
 */
void PathPlan::addPathEdge(int n1, int n2){
    double length = sqrt(
             	pow(fencePosts[n2].x - fencePosts[n1].x, 2) +
             	pow(fencePosts[n2].y - fencePosts[n1].y, 2)
             );
    paths[n1][n2] = length;
    paths[n2][n1] = length;
    
}

/**
 * Get the cross product of two 2d vectors.
 * @param [in] Ax The x value of the first vector.
 * @param [in] Ay The y value of the first vector.
 * @param [in] Bx The x value of the 2nd vector.
 * @param [in] By The y value of the 2nd vector.
 * @return The cross product of the two vectors.
 */
double PathPlan::crossProduct(double Ax, double Ay, double Bx, double By){
    return Ax*By - Ay*Bx;
}

/**
 * Check if two line-segments cross.
 * @param [in] n1 The origin node of the first line segment.
 * @param [in] n2 The terminating node of the first line segment.
 * @param [in] n3 The origin node of the 2nd line segment.
 * @param [in] n4 The terminating node of the 2nd line segment.
 * @return True if the line segments inersect.
 */
bool PathPlan::checkIntersection(node n1, node n2, node n3, node n4){
	////node n1=nodes[node1], n2=nodes[node2], n3=nodes[node3], n4=nodes[node4];
    //if no line exists to intersect, no intersection
    //if(collisionBoundary[node3][node4] < 0) return false;

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

/**
 * Calculates the direct-line distance between 2 nodes
 * @param [in] n1 The origin node.
 * @param [in] n2 The terminating node.
 * @return The displacement between n1 and n2.
 */
double PathPlan::displacement(node n1, node n2){
    return sqrt(
                pow(n2.x - n1.x, 2) +
             	pow(n2.y - n1.y, 2)
           );
}

/**
 * Figures out if a node is inside a collision zone.
 * Will give inconsistent results if the node is on the edge of a collision zone.
 * @param [in] n A node.
 * @return True if the node is inside a collision zone.
 */
bool PathPlan::checkInsidePolygon(node n){
    //raytrace from the node to some arbitrarily distant point. if the number of times the ray intersects an edge is even, then it is inside a polygon.
    node farDistant; farDistant.x =0.0; farDistant.y =0.0;
    bool collides = false;
    int index = 0;
    for(size_t i = 0; i<polygonSides.size(); i++){
        int numCollisions=0;
        for(int j=0; j<polygonSides[i]-1; j++){
            if( collisionBoundary[index][index+1] > 0 && checkIntersection(n, farDistant, nodes[index], nodes[index+1]) ) numCollisions ++;
            index++;
        }
        //check last joining line of the polygon
        if( collisionBoundary[index][index+1-polygonSides[i]] > 0 && checkIntersection(n, farDistant, nodes[index], nodes[index+1-polygonSides[i]]) ) numCollisions ++;
        index++;
                
        if(numCollisions%2 != 0) collides = true;
    }
    if(collides) return true;  
    return false;
    
    
}

/**
 * Generate shortest path between two coordinates that avoids collision zones.
 * @param [in] A the starting coordinate.
 * @param [in] B the ending coordinate.
 * @return A std::deque of indeces indicating a sequence of nodes.
 */
std::deque<int> PathPlan::detour(Coord3D A, Coord3D B ){
	
	node start; start.x = A.lon; start.y = A.lat;
	node end; end.x = B.lon; end.y = B.lat;
	int startIndex = fencePosts.size();
	addFencePost(start.x, start.y);//fencePosts.push_back(start); 
	int endIndex = fencePosts.size();
	addFencePost(end.x, end.y);//fencePosts.push_back(end);
	
	//add new start and end nodes to graph, generate traversable paths to nodes
	for(size_t i =0; i<fencePosts.size(); i++){
		bool traversableFromStart = true;
		bool traversableToEnd = true;
		for(int j =0; j<numNodes; j++){
		    for(int k = j; k<numNodes; k++){
			    if( checkIntersection(start, fencePosts[i], nodes[j], nodes[k]) && collisionBoundary[j][k]>0) traversableFromStart=false;
			    if( checkIntersection(end, fencePosts[i], nodes[j], nodes[k]) && collisionBoundary[j][k]>0)  traversableToEnd=false;
			    if(!traversableFromStart && !traversableToEnd) break;
			}
		}
		if(traversableFromStart) addPathEdge(startIndex, i);
		if(traversableToEnd) addPathEdge(i,endIndex);
	}
	
	//use A* algorithm to find shortest path from start to end
	
	std::vector<double> distance(fencePosts.size());
	std::vector<double>  fitness(fencePosts.size());
	std::vector<int> pathtree(fencePosts.size());	
	std::deque<int> OPEN;
	std::deque<int> CLOSED;
	
	OPEN.push_front(startIndex);
	fitness[startIndex] = displacement(nodes[startIndex], nodes[endIndex]);
	
	//A* main loop
	bool success = false;
	while(OPEN.size() > 0 && !success){
		int n = 0; //index of OPEN with most-fit node
		//find most-fit discovered node
		for(size_t i = 1; i<OPEN.size(); i++){
		    if(fitness[OPEN[i]]<fitness[OPEN[n]]) n=i;
		}
		//check if node is destination
		if(OPEN[n] == endIndex){
		    success = true;
		    break;
		}
		
		//enqueue all of most-fit node's neighbors
		for(size_t i = 0; i< fencePosts.size(); i++){
		    bool explored = false;
		    for(size_t j= 0; j<OPEN.size(); j++) if(OPEN[j]==(int)i) explored = true;
		    for(size_t j= 0; j<CLOSED.size(); j++) if(CLOSED[j]==(int)i) explored = true;
		    		    
		    if(!explored && paths[OPEN[n]][i] > 0){
			    distance[i] = distance[OPEN[n]]+paths[OPEN[n]][i];
			    fitness[i] =distance[i] + displacement(fencePosts[OPEN[n]], fencePosts[i]);
		        pathtree[i] = OPEN[n];
		        OPEN.push_back(i);
		    }
		}
		CLOSED.push_back(OPEN[n]);
		OPEN.erase (OPEN.begin()+n);
	}

    deleteFencePost(endIndex);
    deleteFencePost(startIndex);	
    
    
    //create path from pathtree
    std::deque<int> path;
    int backstep = endIndex;
    while(backstep != startIndex){
        path.push_front(backstep);
        backstep = pathtree[backstep];
    }
    path.push_front(startIndex);
    
    return path;
}

/**
 * Generates traversable paths around existing collision zones.
 */
void PathPlan::generateGraph(){
    
    //Generate graph of permissible nodes ("fenceposts")
    for(size_t i=0; i<nodes.size(); i++){

        node B = nodes[i];      
        node A; node C; 
        double ABLength; double CBLength;
        
        bool AFound = false;
        for(size_t j = 0; j<nodes.size(); j++){
            if(collisionBoundary[i][j] > 0){
                if(!AFound){
                    AFound = true; 
                    A = nodes[j];
                    ABLength = collisionBoundary[i][j];
                }else{
                    C = nodes[j];
                    CBLength = collisionBoundary[i][j];
                }
            }
        }
        double normalVectorX = (B.x-A.x)/ABLength + (B.x-C.x)/CBLength;
        double normalVectorY = (B.y-A.y)/ABLength + (B.y-C.y)/CBLength;
        double normalVectorLength = sqrt( pow(normalVectorX, 2) + pow(normalVectorY, 2) );
        normalVectorX *= errorRadius/normalVectorLength;
        normalVectorY *= errorRadius/normalVectorLength;
        
        
        node n1, n2;
        n1.x = B.x + normalVectorX; n1.y = B.y + normalVectorY;
        n2.x = B.x - normalVectorX; n2.y = B.y - normalVectorY;

        if (!checkInsidePolygon(n1)) addFencePost(n1.x, n1.y);//fencePosts.push_back(n1);
        if (!checkInsidePolygon(n2)) addFencePost(n2.x, n2.y);//fencePosts.push_back(n2);
    }
    
    //generate traversable edges
    for(size_t i = 0; i < fencePosts.size(); i++){      // for permissible node
	    for(size_t j = 0; j < fencePosts.size(); j++){       // with every other fencePost
	    	bool traversable = true;
	        
	        for(size_t k = 0; k < nodes.size(); k++){          //compare with the edges of the collision zones for intersections
	            for(size_t l = k; l < nodes.size(); l++){
	                if(collisionBoundary[k][l] > 0){
	                    if( checkIntersection(fencePosts[i], fencePosts[j], nodes[k], nodes[l]) ){
	                    //if( /*checkIntersection(fencePosts[i], fencePosts[j], fencePosts[k], fencePosts[l]) ||*/ checkIntersection(fencePosts[i], fencePosts[j], nodes[k], nodes[l])){ 
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


/**
 * Creates polygons around all voxels that contain an obstacle
 */
void PathPlan::readGridSpace(){
    /*
    GPSData d;
    fc->gps->GetLatest(&d);
    index3D gridLocation = worldToGrid(Coord3D{d.fix.lat, d.fix.lon, d.fix.alt});    
    double height = gridLocation.z;
    */
    
    for(size_t i = 0; i < gridSpace->grid.size(); i++){
        for(size_t j = 0; j < gridSpace->grid[0].size(); j++){
            for(size_t height = 0; height < gridSpace->grid[1].size()/2; height++){
                if (gridSpace->grid[i][j][height].isFull){
                
                    std::deque<Coord3D> collisionZone; collisionZone.resize(4);
                    collisionZone[0] = gridSpace->gridToWorld( GridSpace::index3D{ (double)i, 	(double)j, 		(double)height } );
                    collisionZone[1] = gridSpace->gridToWorld( GridSpace::index3D{ (double)i+1,	(double)j, 		(double)height } );         
                    collisionZone[2] = gridSpace->gridToWorld( GridSpace::index3D{ (double)i+1, (double)j+1, 	(double)height } );         
                    collisionZone[3] = gridSpace->gridToWorld( GridSpace::index3D{ (double)i, 	(double)j+1, 	(double)height } ); 

                    addPolygon(collisionZone); 
                }
            }
        }
    }    
}




/**
 * Creates a graphic .svg file displaying collision zones, traversable paths, and flightplan.
 * Used for testing purposes. The picopter should never need to call this function.
 */
void PathPlan::writeGraphSVGJamesOval(const char *fileName, std::deque<Waypoints::Waypoint> flightPlan){
    
    double originy = -31.979272, originx = 115.817147;
    double terminy = -31.980967, terminx = 115.818789;
    double width = 417, height = 505;
	
	//write header
	std::ofstream map;
    map.open (fileName);
    map << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n\n<svg\n    id=\"svg6165\"\n    version=\"1.1\"\n    width=\"417\"\n    height=\"505\">\n\n";
    map << "    <image\n     width=\"417\"  \n     height=\"505\" \n     xlink:href=\"file:./Screenshot%20from%202015-09-13%2017:02:44.png\"  \n     id=\"image6173\"\n     x=\"0\"\n     y=\"0\" />\n";
    
    //Draw collision zones
    if(nodes.size() > 0){
        int index = 0;
        for(size_t i = 0; i<polygonSides.size(); i++){
            map << "   <path\n     style=\"fill:#ff0000;stroke:none;stroke-width:1px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1;fill-opacity:1;opacity:0.5\"\n" << "        d=\"M   ";
     
            for(int j=0; j<polygonSides[i]; j++){
                map << (nodes[index].x-originx)/(terminx-originx) * width  << ',' << (nodes[index].y-originy)/(terminy-originy) * height << " ";
                index++;
            }
            map << " Z\"\n     id=\"path3068\"/>\n";
        }
    }
    
    //Draw traversable paths
    if(fencePosts.size() > 0){
        for(size_t i=0; i<fencePosts.size()-1; i++){
            for(size_t j=i+1; j<fencePosts.size(); j++){
                if(paths[i][j] > 0){
                    map << "   <path\n     style=\"fill:none;stroke:#000000;stroke-width:1px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1\"\n     d=\"M ";
                    map << (fencePosts[i].x-originx)/(terminx-originx) * width  << ',' << (fencePosts[i].y-originy)/(terminy-originy) * height;
                    map << ' ';
                    map << (fencePosts[j].x-originx)/(terminx-originx) * width  << ',' << (fencePosts[j].y-originy)/(terminy-originy) * height;
                    map << "\"\n     id=\"path" << i << j << "\"/>\n";
                }
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

/**
 * prints path edge adjacency matrix to console.
 * Used for testing purposes. The picopter should never need to call this function.
 */
void PathPlan::printAdjacencyMatrix(){
	std::cout.precision(4);
    for(size_t i=0; i< nodes.size(); i++){        
        for(size_t j=0; j< nodes.size(); j++){
            std::cout << std::fixed << collisionBoundary[i][j] << " ";
        }
        std::cout << "\n";
    }
}
