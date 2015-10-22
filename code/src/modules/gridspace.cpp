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
GridSpace::GridSpace( FlightController *fc )
: grid (128,vector<vector<voxel> >(128,vector <voxel>(128)))
{
    double copterDiameter = 1.5; //metres
    double copterHeight   = 3.0; //metres
    
    //get starting GPS position
    //fc->fb->GetHomePosition(&launchPoint);
    GPSData d;
    fc->gps->WaitForFix();
    fc->gps->GetLatest(&d);
    assert(!std::isnan(d.fix.lat) && !std::isnan(d.fix.lon) && !std::isnan(d.fix.alt));
    launchPoint = Coord3D{d.fix.lat, d.fix.lon, d.fix.alt};
    
    launchIndex = index3D{(double)grid.size()/2, (double)grid[0].size()/2, (double)grid[0][0].size()/2};


    Coord3D DiamNorth =  navigation::CoordAddOffset( launchPoint, navigation::Point3D{0, copterDiameter, 0} );
	voxelLength = abs( DiamNorth.lat - launchPoint.lat);
	
	Coord3D DiamEast =  navigation::CoordAddOffset( launchPoint, navigation::Point3D{copterDiameter, 0, 0} );
	voxelWidth = abs( DiamEast.lon - launchPoint.lon);
	
	//Coord3D DiamDown =  navigation::CoordAddOffset( launchPoint, navigation::Point3D{0, 0, copterHeight} );
	//voxelHeight = abs( DiamDown.alt - launchPoint.alt);
    voxelHeight = copterHeight;
    
    //voxelWidth *= 0.001;
    //voxelWidth *= 0.001;
    
    
}




GridSpace::index3D GridSpace::findEndPoint(FlightController *fc){

    if (fc->lidar){        
       
        double lidarm = fc->lidar->GetLatest() / 100.0;
        Vec3d ray(0, 0, lidarm);                                                //Vector representing the lidar ray
        
        Matx33d MLidar = rotationMatrix(-6,-3,0);                               //the angle between the camera and the lidar (deg)

 //only works if GetGimbalPose() works
 /*        
        EulerAngle gimbal;
        fc->fb->GetGimbalPose(&gimbal);
        Matx33d Mbody = rotationMatrix(gimbal.roll, gimbal.pitch, gimbal.yaw);  //find the transformation matrix from camera frame to the body.
*/
        //assume Gimbal isn't working: fix to point straight ahead.
        Matx33d Mbody = rotationMatrix(0.0, 90, 0.0);
        
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
    
    if (!fc->gps->HasFix()) {
		return;
	}
    GPSData d;
    fc->gps->GetLatest(&d);
    assert(!std::isnan(d.fix.lat) && !std::isnan(d.fix.lon) && !std::isnan(d.fix.alt));
    index3D startPoint = worldToGrid(Coord3D{d.fix.lat, d.fix.lon, d.fix.alt});
    index3D endPoint   = findEndPoint(fc);
/*
 std::cout << "(" << startPoint.x << ", " << startPoint.y << ", " << startPoint.z << "), " << "(" << endPoint.x << ", " << endPoint.y << ", " << endPoint.z << ")\n";
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
    loc.x = (GPSloc.lon - launchPoint.lon)/voxelLength + launchIndex.x;
    loc.y = (GPSloc.lat - launchPoint.lat)/voxelWidth  + launchIndex.y;
    loc.z = (GPSloc.alt - launchPoint.alt)/voxelHeight + launchIndex.z;
    
    return loc;
}

Coord3D GridSpace::gridToWorld(index3D loc){
    
    Coord3D GPSloc;
    GPSloc.lon = (loc.x-launchIndex.x)*voxelLength + launchPoint.lon;
    GPSloc.lat = (loc.y-launchIndex.y)*voxelWidth  + launchPoint.lat;
    GPSloc.alt = (loc.z-launchIndex.z)*voxelHeight + launchPoint.alt;
    
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

