/**
 * @file pathtest.cpp
 * @brief Obstacle avoidance testing code
 */

#include "picopter.h"
#include "pathplan.h"
#include "gridspace.h"
using namespace picopter;
using namespace picopter::navigation;
using namespace std;

int main(int argc, char *argv[]) {
    LogInit();
    FlightController fc;
    GridSpace world(&fc);
	PathPlan pathPlan(&world);
    
    
    Coord3D coords[14];
        coords[0].lat=-31.979570497849565;  coords[0].lon=115.817621648311615;
        coords[1].lat=-31.9795181694098289; coords[1].lon=115.817830860614777;
        coords[2].lat=-31.9797365835693199; coords[2].lon=115.81787109375;
        coords[3].lat=-31.9798730921550032; coords[3].lon=115.81762433052063;
        coords[4].lat=-31.9797411338587665; coords[4].lon=115.817543864250183;
        
        coords[5].lat=-31.9796319268494216; coords[5].lon=115.8182492852211;
        coords[6].lat=-31.9798480655961725; coords[6].lon=115.818488001823425;
        coords[7].lat=-31.98002097622701;   coords[7].lon=115.818327069282532;
        coords[8].lat=-31.9797183824092031; coords[8].lon=115.818088352680206;
        
        coords[9].lat=-31.9801142568267807;  coords[9].lon=115.817728936672211;
        coords[10].lat=-31.980218912996591;  coords[10].lon=115.817404389381409;
        coords[11].lat=-31.980432775233254;  coords[11].lon=115.817836225032806;
        
        coords[12].lat=-31.979812;           coords[12].lon=115.817324;
        coords[13].lat=-31.979779;           coords[13].lon=115.818586;
        
        coords[14].lat=-31.9798319268494216; coords[14].lon=115.8182492852211;
        coords[15].lat=-31.9800480655961725; coords[15].lon=115.818488001823425;
        coords[16].lat=-31.98022097622701;   coords[16].lon=115.818327069282532;
        coords[17].lat=-31.9799183824092031; coords[17].lon=115.818088352680206;
        
    deque<Coord3D> czone[4];
    for(int i=0;i<5;i++){
        czone[0].push_front(coords[i]);
    }
    for(int i=5;i<9;i++){
	    czone[1].push_front(coords[i]);
    }
    for(int i=9;i<12;i++){
	    czone[2].push_front(coords[i]);
    }
    for(int i=14;i<18;i++){
	    czone[3].push_front(coords[i]);
    }
    //Waypoints::Waypoint newWaypoint = templateWaypoint;
     //   newWaypoint.pt = coords[i];
	   // czone[0].push_front(newWaypoint);

    /*pathPlan.addPolygon(czone[0]);
    pathPlan.addPolygon(czone[1]);
    pathPlan.addPolygon(czone[2]);
    pathPlan.addPolygon(czone[3]);*/
    
    deque<Waypoints::Waypoint> waypoints;
    
    Waypoints::Waypoint waypoint2;
    waypoint2.roi = coords[13];
    waypoint2.has_roi = true;
    waypoint2.pt = coords[13];
    waypoints.push_front(waypoint2);
    
    Waypoints::Waypoint waypoint1;
    waypoint1.roi = coords[12];
    waypoint1.has_roi = true;
    waypoint1.pt = coords[12];
    waypoints.push_front(waypoint1);   
    
    
    //pathPlan.generateFlightPlan(waypoints);
    pathPlan.writeGraphSVGJamesOval("map.svg",pathPlan.generateFlightPlan(waypoints));

    //pathPlan.printAdjacencyMatrix();
    
    return 0;
}
