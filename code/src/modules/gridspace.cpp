/**
 * @file gridspace.cpp
 * @brief Obstacle describes obstacles in space using a voxel system.
 **/

#include "common.h"
#include "gridspace.h"

#include <math.h>
#include <iostream>

using namespace std;
using namespace picopter;
using namespace picopter::navigation;

GridSpace::GridSpace(/*FLightController *fc*/)
: grid (64,vector<vector<voxel> >(64,vector <voxel>(64)))
{
    double copterRadius = 3.0; //metres
    double copterHeight = 3.0; //metres
    
    launchPoint = getGPS();
    
    double earthRadius = 6378137.00; //metres
    double R2 = earthRadius * cos( degToRad(launchPoint.lat) );
    
    voxelLength = 2*asin(copterRadius / (2*earthRadius) ); //the north-south distance increment, equal to the fuzzyCopter diameter, expressed in geographic degrees.
    voxelWidth = 2*asin(copterRadius / (2*R2) ); //the East-West distance increment, equal to the fuzzyCopter diameter, expressed in geographic degrees.
    voxelHeight = copterHeight;
    
}

void GridSpace::raycast(/*FLightController *fc*/){
    //GPSData d;
    //fc->gps->GetLatest(&d);
    //index3D startPoint = worldToGrid(Coord3D{d.fix.lat, d.fix.lon, d.fix.alt});
    //IMUData imu;
    //fc->imu->GetLatest(&imu);
    //imu.pitch, imu.roll, imu.yaw
    //EulerAngle gimbal;
    //fc->fb->GetGimbalPose(&gimbal);
    //if (fc->lidar) {
    //    double lidarm = fc->lidar->GetLatest() / 100.0;
    //}
    index3D startPoint = worldToGrid(getGPS());
    startPoint.x += 5.0;
    startPoint.y += 3.0;
    index3D endPoint = startPoint; 
    endPoint.x -= 5.0;
    endPoint.y -= 3.0;
    
    double dx = endPoint.x-startPoint.x;
    double dy = endPoint.y-startPoint.y;
    double dz = endPoint.z-startPoint.z;
    
    int window[3]= {(int)startPoint.x, (int)startPoint.y, (int)startPoint.z}; 
    
    //if the line is longest along the X-dimension
    if( abs(dx) >= abs(dy) && abs(dx) >= abs(dz) ){
        double errY = 2*abs(dy)-abs(dx);
        double errZ = 2*abs(dz)-abs(dx);
        int rasterLength = abs(dx);
        for(int i = 0; i < rasterLength; i++){
             grid[window[0]][window[1]][window[2]].observations++;             
             if(errY >0) { 
                window[1]+= (dy < 0) ? -1 : 1;
                errY -= abs(dx)*2;
             }
             if(errZ >0) { 
                window[2]+= (dz < 0) ? -1 : 1;
                errZ -= abs(dx)*2;
             }
             errY += 2*abs(dy);
             errZ += 2*abs(dz);
             window[0] += (dx < 0) ? -1 : 1;
         }
    }else if( abs(dy) >= abs(dx) && abs(dy) >= abs(dz) ){
        double errX = 2*abs(dx)-abs(dy);
        double errZ = 2*abs(dz)-abs(dy);
        int rasterLength = abs(dy);
        for(int i = 0; i < rasterLength; i++){
             grid[window[0]][window[1]][window[2]].observations++;             
             if(errX >0) { 
                window[0]+= (dx < 0) ? -1 : 1;
                errX -= abs(dy)*2;
             }
             if(errZ >0) { 
                window[2]+= (dz < 0) ? -1 : 1;
                errZ -= abs(dy)*2;
             }
             errX += 2*abs(dx);
             errZ += 2*abs(dz);
             window[1] += (dy < 0) ? -1 : 1;
         }
    }else if( abs(dz) >= abs(dx) && abs(dz) >= abs(dy) ){
        double errX = 2*abs(dx)-abs(dz);
        double errY = 2*abs(dy)-abs(dz);
        int rasterLength = abs(dz);
        for(int i = 0; i < rasterLength; i++){
             grid[window[0]][window[1]][window[2]].observations++;             
             if(errX >0) { 
                window[0]+= (dx < 0) ? -1 : 1;
                errX -= abs(dz)*2;
             }
             if(errY >0) { 
                window[1]+= (dy < 0) ? -1 : 1;
                errY -= abs(dz)*2;
             }
             errX += 2*abs(dx);
             errY += 2*abs(dy);
             window[2] += (dz < 0) ? -1 : 1;
         }
    }
    
    //update endpoint location
    grid[window[0]][window[1]][window[2]].observations++;
    
    
}

Coord3D GridSpace::getGPS(){   
    Coord3D pt; 
    pt.lat = -31.979570497849565;  pt.lon=115.817621648311615; pt.alt = 0.0;
    return pt;

}

GridSpace::index3D GridSpace::worldToGrid(Coord3D GPSloc){
    
    index3D loc;
    loc.x = (GPSloc.lon - launchPoint.lon)/voxelLength;
    loc.y = (GPSloc.lat - launchPoint.lat)/voxelWidth;
    loc.z = (GPSloc.alt - launchPoint.alt)/voxelHeight;
    
    return loc;
}

double GridSpace::degToRad(double deg){
    return deg * 3.14159265358979323846/180.00;
}

void GridSpace::printToConsole(int rangeMin, int rangeMax, int zDepth){
    for(int i= rangeMin; i < rangeMax; i++){
        for(int j = rangeMin; j< rangeMax; j++){
            cout << grid[i][j][zDepth].observations << " ";
        }
        cout <<"\n";
    }
}
