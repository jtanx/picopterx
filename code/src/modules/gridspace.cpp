/**
 * @file gridspace.cpp
 * @brief Obstacle describes obstacles in space using a voxel system.
 **/

#include "common.h"
#include "gridspace.h"

#include <math.h>

using namespace picopter;
using namespace picopter::navigation;

GridSpace::GridSpace(){
    copterRadius = 3.0; //metres
    copterHeight = 3.0; //metres
    
    launchPoint = getGPS();
    
    double earthRadius = 6378137.00; //metres
    double R2 = earthRadius * cos( degToRad(launchPoint.lat) );
    
    double voxelLength = 2*asin(copterRadius / (2*earthRadius) ); //the north-south distance increment, equal to the fuzzyCopter diameter, expressed in geographic degrees.
    
}

Coord3D GridSpace::getGPS(){   
    Coord3D pt; 
    pt.lat -31.979570497849565;  pt.lon=115.817621648311615; pt.alt = 0.0;
    return pt;

}

GridSpace::index3D GridSpace::findCopter(){
    Coord3D GPSloc = getGPS();
    
    index3D loc;
    loc.x = (GPSloc.lon - launchPoint.lon)/copterRadius;
    loc.y = (GPSloc.lat - launchPoint.lat)/copterRadius;
    loc.z = (GPSloc.alt - launchPoint.alt)/copterHeight;
    
    return loc;
}

double GridSpace::degToRad(double deg){
    return deg * 3.14159265358979323846/180.00;
}
