/**
 * @file gridspace.cpp
 * @brief Obstacle describes obstacles in space using a voxel system.
 **/

#include "common.h"
#include "gridspace.h"

#include <math.h>
#include <iostream>

using namespace std;
using namespace cv;
using namespace picopter;
using namespace picopter::navigation;


/** 
 * Constructor. Constructs with default settings.
 */
GridSpace::GridSpace(PathPlan *p/*, FlightController *fc*/)
: grid (64,vector<vector<voxel> >(64,vector <voxel>(64)))
{
    pathPlan = p;
    double copterRadius = 3.0; //metres
    double copterHeight = 3.0; //metres
    
    launchPoint = getGPS();
    
    double earthRadius = 6378137.00; //metres
    double R2 = earthRadius * cos( degToRad(launchPoint.lat) );
    
    voxelLength = 2*asin(copterRadius / (2*earthRadius) ); //the north-south distance increment, equal to the fuzzyCopter diameter, expressed in geographic degrees.
    voxelWidth = 2*asin(copterRadius / (2*R2) ); //the East-West distance increment, equal to the fuzzyCopter diameter, expressed in geographic degrees.
    voxelHeight = copterHeight;

voxelWidth *= 100;
voxelLength *= 100;    
    
}


/*
Index3D GridSpace::findEndPoint(FlightController *fc){
    I
    Matx33d MLidar = rotationMatrix(-6,-3,0);   //the angle between the camera and the lidar (deg)
    //find the transformation matrix from camera frame to ground.
    Matx33d Mbody = GimbalToBody(gimbal);
    //Matx33d Mstable = BodyToLevel(imu_data);
    //Matx33d MYaw = LevelToGround(imu_data);
    Matx33d MGnd = BodyToGround(imu_data);
}

*/

void GridSpace::raycast(/*FlightController *fc*/){
    
    
    //IMUData imu;
    //fc->imu->GetLatest(&imu);
    ////imu.pitch, imu.roll, imu.yaw
    //EulerAngle gimbal;
    //fc->fb->GetGimbalPose(&gimbal);
    //if (fc->lidar) double lidarm = fc->lidar->GetLatest() / 100.0;
    double lidar = 2.0;
    
    //GPSData d;
    //fc->gps->GetLatest(&d);
    //index3D startPoint = worldToGrid(Coord3D{d.fix.lat, d.fix.lon, d.fix.alt});
    index3D startPoint = worldToGrid(getGPS());
    index3D endPoint = startPoint; 
    endPoint.x += 1.0;
    endPoint.y += 1.0;
    
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
    if(lidar > 0 && grid[window[0]][window[1]][window[2]].isFull==false){
    
cout << "Requesting collision zone: ";
         
         grid[window[0]][window[1]][window[2]].isFull=true;
         deque<Coord3D> collisionZone; collisionZone.resize(4);
         collisionZone[0] = gridToWorld( index3D{ (double)window[0], (double)window[1], (double)window[2] } );
cout << " (" << collisionZone[0].lat << ", " << collisionZone[0].lon << ", " << collisionZone[0].alt << "), ";
         collisionZone[1] = gridToWorld( index3D{ (double)(window[0]+1), (double)window[1], (double)window[2]} );         
cout << " (" << collisionZone[1].lat << ", " << collisionZone[1].lon << ", " << collisionZone[1].alt << "), ";
         collisionZone[2] = gridToWorld( index3D{ (double)(window[0]+1), (double)(window[1]+1), (double)window[2]} );         
cout << " (" << collisionZone[2].lat << ", " << collisionZone[2].lon << ", " << collisionZone[2].alt << "), ";
         collisionZone[3] = gridToWorld( index3D{ (double)window[0], (double)(window[1]+1), (double)window[2]} );         
cout << " (" << collisionZone[3].lat << ", " << collisionZone[3].lon << ", " << collisionZone[3].alt << ")\n";
         
         pathPlan->addPolygon(collisionZone);
         
 
          
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
    
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    imwrite("alpha.png", m, compression_params);   
}
/*

void rasterDistrib(Mat &mat, Distrib *dist)
{
    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {
                Vec4b& rgba = mat.at<Vec4b>(i, j);
                Vec3d B(j-(mat.cols/2.0), i-(mat.rows/2.0),0);
                rgba[0] = saturate_cast<uchar>(sampleDistrib(dist, &B) * UCHAR_MAX);
                rgba[1] = 0;//rgba[0];
                rgba[2] = 0;//rgba[0];
                rgba[3] = UCHAR_MAX;

                
                //rgba[0] = UCHAR_MAX;
                //rgba[1] = saturate_cast<uchar>((float (mat.cols - j)) / ((float)mat.cols) * UCHAR_MAX);
                //rgba[2] = saturate_cast<uchar>((float (mat.rows - i)) / ((float)mat.rows) * UCHAR_MAX);
            //rgba[3] = saturate_cast<uchar>(0.5 * (rgba[1] + rgba[2]));

        }
    }
}

void storeDistrib(Distrib* dist, int rows, int cols){
    Mat mat(rows, cols, CV_8UC4);
    rasterDistrib(mat,dist);

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    imwrite("alpha.png", mat, compression_params);

    std::cout <<  "Saved PNG file with alpha data." << std::endl;
}
*/
