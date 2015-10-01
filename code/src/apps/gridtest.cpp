/**
 * @file gridtest.cpp
 * @brief voxel world testing code
 */

#include "picopter.h"
#include "gridspace.h"
using namespace picopter;
using namespace picopter::navigation;
using namespace std;

int main(int argc, char *argv[]) {
/*
    double originy = -31.979272, originx = 115.817147;
    double terminy = -31.980967, terminx = 115.818789;
    cout << "bounding coordinates:      ";
    cout << " (" << originy << ", " << originx << ", " << 0 << "), ";
    cout << " (" << terminy << ", " << terminx << ", " << 0 << "), \n";
    
    
    PathPlan pathPlan;
    
    //FlightController fc;
	GridSpace world( &pathPlan );
	//world.printToConsole(0, 8, 0);
	//cout << "\n";
	//world.raycast(fc);world.raycast();world.raycast();world.raycast();world.raycast();world.raycast();
	world.writeImage();
	
	Coord3D coords[5];
        coords[0].lat=-31.979570497849565;  coords[0].lon=115.817621648311615;
        coords[1].lat=-31.9795181694098289; coords[1].lon=115.817830860614777;
        coords[2].lat=-31.9797365835693199; coords[2].lon=115.81787109375;
        coords[3].lat=-31.9798730921550032; coords[3].lon=115.81762433052063;
        coords[4].lat=-31.9797411338587665; coords[4].lon=115.817543864250183;
    std::deque<Coord3D> czone[4];
    for(int i=0;i<5;i++){
        czone[0].push_front(coords[i]);
    }
	pathPlan.addPolygon(czone[0]);
	
	std::deque<Waypoints::Waypoint> waypoints;
    
    Waypoints::Waypoint waypoint2;
    waypoint2.roi = Coord3D{-31.979779, 115.818586, 10.0};
    waypoint2.has_roi = true;
    waypoint2.pt = Coord3D{-31.979779, 115.818586, 10.0};
    waypoints.push_front(waypoint2);
    
    Waypoints::Waypoint waypoint1;
    waypoint1.roi = Coord3D{-31.979812, 115.817324};
    waypoint1.has_roi = true;
    waypoint1.pt = Coord3D{-31.979812, 115.817324};
    waypoints.push_front(waypoint1);   
    
    
    //pathPlan.generateFlightPlan(waypoints);
    pathPlan.writeGraphSVGJamesOval("map.svg",pathPlan.generateFlightPlan(waypoints));
	
	//cout << "\n\n";
	//pathPlan.printAdjacencyMatrix();*/
	return 0;
}
