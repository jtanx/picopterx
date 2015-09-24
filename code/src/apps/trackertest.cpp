/**
* @file trackertest.cpp
* @brief Object tracker Code tests
*/
#include "picopter.h"
#include "object_tracker.h"
#include "observations.h"

using namespace picopter;
using namespace picopter::navigation;


void printMatrix(cv::Matx33d mat){
    for(int i=0;i<3;i++){
        std::cout << "[";
        for(int j=0;j<3;j++){
            std::cout << mat(i,j);
            if(j!=2) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
}

void printVector(cv::Matx31d mat){
    for(int i=0;i<3;i++){
        std::cout << "[" << mat(i,0) << "]" << std::endl;
    }
}
void printCoord3d(Coord3D coord){
    std::cout << "[" << coord.lat << "]" << std::endl;
    std::cout << "[" << coord.lon << "]" << std::endl;
    std::cout << "[" << coord.alt << "]" << std::endl;
}

int main(int argc, char *argv[]) {

    std::cout << "Test Rotation Matrix: 30 deg roll" << std::endl;

    cv::Matx33d rot = rotationMatrix(30,0,0);
    printMatrix(rot);

    std::cout << "Test Base Distribution" << std::endl;
    Distrib A = generatedistrib();
    printMatrix(A.axes);
    printVector(A.vect);
    std::cout << "Translate Distrib by x=10" << std::endl;
    Distrib B = translateDistrib(A, 10, 0, 0);
    printMatrix(B.axes);
    printVector(B.vect);
    std::cout << "Stretch Distrib by y=5" << std::endl;
    Distrib C = stretchDistrib(B, 1, 5, 1);
    printMatrix(C.axes);
    printVector(C.vect);
    std::cout << "Rotate Distrib by yaw 30 deg" << std::endl;
    Distrib D = rotateDistrib(C, rotationMatrix(0,0,30));
    printMatrix(D.axes);
    printVector(D.vect);
    std::cout << "Merge Distribs" << std::endl;
    Distrib E = combineDistribs(D,C);
    printMatrix(E.axes);
    printVector(E.vect);
    std::cout << "Vector Sum" << std::endl;
    Distrib F = vectorSum(D,E);
    printMatrix(F.axes);
    printVector(F.vect);
    //B = rotateDistrib(stretchDistrib(A, 1, 2, 1), rotationMatrix(0,0,30));
    //C = stretchDistrib(A, 2, 1, 1);

    std::cout << "Test NED ground coords" << std::endl;

    ObjectTracker testTracker;
    Coord3D launch;
    launch.lat = -32;
    launch.lon = 115;
    launch.alt = 0;
    testTracker.launch_point = launch;
    std::cout << "Launch From" << std::endl;
    printCoord3d(testTracker.launch_point);
    
    std::cout << "Convert to ground coords" << std::endl;
    cv::Matx31d gndlaunch = testTracker.GroundFromGPS(testTracker.launch_point);
    printVector(gndlaunch);

    std::cout << "Move in ground coords" << std::endl;
    gndlaunch += cv::Matx31d(10,5,-2);
    printVector(gndlaunch);

    std::cout << "Convert back to GPS" << std::endl;
    Coord3D testLoc = testTracker.GPSFromGround(gndlaunch);
    printCoord3d(testLoc);



    return 0;
}