/**
 * @file gridspace.cpp
 * @brief Obstacle describes obstacles in space using a voxel system.
 **/

#include "common.h"
#include "gridspace.h"
#include "observations.h"

#include <math.h>
#include <iostream>

using namespace std;
using namespace cv;
using namespace picopter;
using namespace picopter::navigation;


/** 
 * Constructor. Constructs with default settings.
 */
GridSpace::GridSpace(FlightController *fc)
: grid (64,vector<vector<voxel> >(64,vector <voxel>(64)))
{
    double copterRadius = 3.0; //metres
    double copterHeight = 3.0; //metres
    
    fc->fb->GetHomePosition(&launchPoint);
    
    double earthRadius = 6378137.00; //metres
    double R2 = earthRadius * cos( degToRad(launchPoint.lat) );
    
    voxelLength = 2*asin(copterRadius / (2*earthRadius) ); //the north-south distance increment, equal to the fuzzyCopter diameter, expressed in geographic degrees.
    voxelWidth = 2*asin(copterRadius / (2*R2) ); //the East-West distance increment, equal to the fuzzyCopter diameter, expressed in geographic degrees.
    voxelHeight = copterHeight;

voxelWidth *= 100;
voxelLength *= 100;    
    
}



GridSpace::index3D GridSpace::findEndPoint(FlightController *fc){

    if (fc->lidar){        
        
        double lidarm = fc->lidar->GetLatest() / 100.0;
        Vec3d ray(0, 0, lidarm);                                                //Vector representing the lidar ray
        
        Matx33d MLidar = rotationMatrix(-6,-3,0);                               //the angle between the camera and the lidar (deg)
        
        EulerAngle gimbal;
        fc->fb->GetGimbalPose(&gimbal);
        Matx33d Mbody = rotationMatrix(gimbal.roll, gimbal.pitch, gimbal.yaw);  //find the transformation matrix from camera frame to the body.
        
        IMUData imu;
        fc->imu->GetLatest(&imu);
        Matx33d MGnd = rotationMatrix(imu.roll, imu.pitch, imu.yaw);            //find the transformation matrix from the body to the ground.
        
        //apply rotations to ray
        ray =   MGnd *  Mbody * MLidar * ray;
        
        GPSData d;
        fc->gps->GetLatest(&d);
          
        return worldToGrid( navigation::CoordAddOffset( navigation::Coord3D{d.fix.lat, d.fix.lon, d.fix.alt}, navigation::Point3D{ray(0), ray(1), ray(2)} ) );
    }
    
    index3D voxelLoc{ 0,0,0};
    return voxelLoc;
}



void GridSpace::raycast(FlightController *fc){
    std::lock_guard<std::mutex> lock(mutex);
    if ( !(fc->lidar) ) cout << "no lidar. \n";
    
    GPSData d;
    fc->gps->GetLatest(&d);
    index3D startPoint = worldToGrid(Coord3D{d.fix.lat, d.fix.lon, d.fix.alt});
    index3D endPoint   = findEndPoint(fc);
    /*
    index3D startPoint = worldToGrid(getGPS());
    index3D endPoint = startPoint; 
    endPoint.x += 1.0;
    endPoint.y += 1.0;
    */
    
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
    }else if( abs(dy) >= abs(dx) && abs(dy) >= abs(dz) ){ //if the line is longest along the Y-dimension
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
    }else if( abs(dz) >= abs(dx) && abs(dz) >= abs(dy) ){ //if the line is longest along the Z-dimension
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
    
    
    //fill voxel with observation
    double lidarm = fc->lidar->GetLatest() / 100.0;
    if(lidarm > 0 && grid[window[0]][window[1]][window[2]].isFull==false){
    

         
         grid[window[0]][window[1]][window[2]].isFull=true;
         
/*         
         deque<Coord3D> collisionZone; collisionZone.resize(4);
         collisionZone[0] = gridToWorld( index3D{ (double)window[0], (double)window[1], (double)window[2] } );

         collisionZone[1] = gridToWorld( index3D{ (double)(window[0]+1), (double)window[1], (double)window[2]} );         

         collisionZone[2] = gridToWorld( index3D{ (double)(window[0]+1), (double)(window[1]+1), (double)window[2]} );         

         collisionZone[3] = gridToWorld( index3D{ (double)window[0], (double)(window[1]+1), (double)window[2]} );         

         
         pathPlan->addPolygon(collisionZone);     
*/ 
    }
  
}

Coord3D GridSpace::getGPS(){   
    Coord3D pt; 
    pt. lat=( -31.979272 + -31.980967 )/2;    
    pt.lon=(115.817147 + 115.818789)/2;   
    pt.alt = 2.0;
    return pt;

}

GridSpace::index3D GridSpace::worldToGrid(Coord3D GPSloc){
    index3D loc;
    loc.x = (GPSloc.lon - launchPoint.lon)/voxelLength + 31.00;
    loc.y = (GPSloc.lat - launchPoint.lat)/voxelWidth  + 31.00;
    loc.z = (GPSloc.alt - launchPoint.alt)/voxelHeight + 31.00;
    
    return loc;
}

Coord3D GridSpace::gridToWorld(index3D loc){
    
    Coord3D GPSloc;
    GPSloc.lon = voxelLength*(loc.x-31.00) + launchPoint.lon;
    GPSloc.lat = voxelWidth*(loc.y-31.00) + launchPoint.lat;
    GPSloc.alt = voxelHeight*(loc.z-31.00) + launchPoint.alt;
    
    return GPSloc;
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


void GridSpace::writeImage(){
    int bravado = 8;
    Mat m(64, 64, CV_8UC4);
    for(int i=0; i<64; i++){
        for(int j=0; j<64; j++){
            int sum = 0;
            for(int k=0; k<64; k++) sum += grid[i][j][k].observations;
            if(sum/bravado >= 1) sum = bravado;
            sum = UCHAR_MAX * sum / bravado;
            
            Vec4b& rgba = m.at<Vec4b>(i, j);
            rgba[0] = sum;
            rgba[1] = sum;
            rgba[2] = sum;
            rgba[3] = UCHAR_MAX; 
         }
    } 
    
    vector<int> compression_params{CV_IMWRITE_PNG_COMPRESSION, 9};
    //compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    //compression_params.push_back(9);
    
    std::string name = GenerateFilename(
        PICOPTER_HOME_LOCATION "/pics", "gridspace_alpha", ".png");
    imwrite(name, m, compression_params);   
}

