/**
* @file trackertest.cpp
* @brief Object tracker Code tests
*/
#include "picopter.h"
#include "object_tracker.h"
#include "observations.h"

using namespace picopter;
using namespace cv;
using namespace picopter::navigation;
using std::chrono::steady_clock;
using std::chrono::milliseconds;
using std::chrono::microseconds;
using std::chrono::seconds;
using std::chrono::duration_cast;
using std::this_thread::sleep_for;
void printMatrix(Matx33d mat){
    for(int i=0;i<3;i++){
        std::cout << "[";
        for(int j=0;j<3;j++){
            std::cout << mat(i,j);
            if(j!=2) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
}

void printVector(Vec3d mat){
    for(int i=0;i<3;i++){
        std::cout << "[" << mat(i) << "]" << std::endl;
    }
}
void printCoord3d(Coord3D coord){
    std::cout << "[" << coord.lat << "]" << std::endl;
    std::cout << "[" << coord.lon << "]" << std::endl;
    std::cout << "[" << coord.alt << "]" << std::endl;
}



int main(int argc, char *argv[]) {
    
    //CLOCK_TYPE m_task_start = steady_clock::now();

    std::cout << "Test Rotation Matrix: 30 deg roll" << std::endl;

    Matx33d rot = rotationMatrix(30,0,0);
    printMatrix(rot);
    /*
    std::cout << "Test Base Distribution" << std::endl;
    Distrib A = generatedistrib();
    printMatrix(A.axes);
    printVector(A.vect);
    std::cout << "Translate Distrib by x=10" << std::endl;
    Distrib B = translateDistrib(A, 10, 0, 30);
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
    */
    //B = rotateDistrib(stretchDistrib(A, 1, 2, 1), rotationMatrix(0,0,30));
    //C = stretchDistrib(A, 2, 1, 1);
//    Mat mat(rows, cols, CV_8UC4);
//    storeDistrib(&mat, &F, "alpha.png");

    std::cout << "Test broken Distribs" << std::endl;
    Matx33d brokenAxesA (
        0, 0, 0,
        0, 0, 0,
        0, 0, 0); //totally unknown
    Vec3d brokenVectA (0,0,0);
    Distrib brokenDistribA = {brokenAxesA,brokenVectA};
    Matx33d brokenAxesB (
        0, 0, 0,
        0, 0, 0,
        0, 0, 0); //totally unknown
    Vec3d brokenVectB (0,0,0);
    Distrib brokenDistribB = {brokenAxesB,brokenVectB};

    Distrib brokenDistribC = combineDistribs(brokenDistribA,brokenDistribB);

    printMatrix(brokenDistribC.axes);
    printVector(brokenDistribC.vect);


/*

    Mat mat(240, 320, CV_8UC4);

    std::string filename = "alpha.png";
    Vec4b colour(UCHAR_MAX,0,0,UCHAR_MAX);
    //colour[0] = UCHAR_MAX;
    //colour[1] = UCHAR_MAX;
    //colour[2] = UCHAR_MAX;
    rasterDistrib(&mat, &F, colour, 1.0);
    //mat = Mat::zeros(mat.rows, mat.cols, CV_8UC4);
    colour = Vec4b(0,UCHAR_MAX,0,UCHAR_MAX);
    rasterDistrib(&mat, &A, colour, 1.0);
    storeDistrib(&mat, filename);

*/
    /*
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
    Vec3d gndlaunch = testTracker.GroundFromGPS(testTracker.launch_point);
    printVector(gndlaunch);

    std::cout << "Move in ground coords" << std::endl;
    gndlaunch += Vec3d(10,5,-10);
    printVector(gndlaunch);

    std::cout << "Convert back to GPS" << std::endl;
    Coord3D testLoc = testTracker.GPSFromGround(gndlaunch);
    printCoord3d(testLoc);

    std::cout << "And back to ground coords" << std::endl;
    Vec3d testCoord = testTracker.GroundFromGPS(testLoc);
    printVector(testCoord);

    std::cout << "Test Observations" << std::endl;

    GPSData gps_pos;
    gps_pos.fix.lat = testLoc.lat;
    gps_pos.fix.lon = testLoc.lon;
    gps_pos.fix.alt = testLoc.alt;

    EulerAngle gimbal;
    gimbal.roll = 0;
    gimbal.pitch = 10;
    gimbal.yaw = 0;

    IMUData imu_data;
    imu_data.roll = 10;
    imu_data.pitch = 0;
    imu_data.yaw = 0;

    ObjectInfo object;
    object.image_width = 320;
    object.position.y = 100;
    object.position.x = 0;

    std::cout << "Initialise cam observation" << std::endl;
    Observation firstSighting = testTracker.ObservationFromImageCoords(m_task_start-steady_clock::now(), &gps_pos, &gimbal, &imu_data, &object);
    printMatrix(firstSighting.location.axes);
    printVector(firstSighting.location.vect);

    std::cout << "Initialise ground assumption" << std::endl;
    Observations new_thing(testTracker.AssumptionGroundLevel());    //start with the assumption
    printMatrix(new_thing.getLocation().axes);
    printVector(new_thing.getLocation().vect);

//    std::cout << "combination determinant = " << determinant(new_thing.getLocation().axes + firstSighting.location.axes) << std::endl;

    std::cout << "Append Observation" << std::endl;
    new_thing.appendObservation(firstSighting);
    printMatrix(new_thing.getLocation().axes);
    printVector(new_thing.getLocation().vect);

    std::cout << "Test Vantage Point" << std::endl;
    Coord3D vantage = testTracker.CalculateVantagePoint(&gps_pos, &new_thing, true);
    printVector(testTracker.GroundFromGPS(vantage));

*/
    //Distrib ntloc = new_thing.getLocation();


    return 0;
}