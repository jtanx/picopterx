/**
 * @file observations.cpp
 * @brief Utility functions for manipulating and tracking uncertainties in observations
 */




#include "common.h"
#include "observations.h"

using namespace picopter;
using namespace cv;
Observations::Observations() {

}


//estimate the probability the given observation is of the same object
double getSameProbability(Observation* observation){    

return 0;
}

//calculate the centre and covariance widths of this object
DistribParams getDistribParams(){                         

    DistribParams D = {0};
    return D;
}

//add another sighting to this object
void Observations::appendObservation(Observation* observation){     
    sightings.push_back(observation);
    //for(int i=0; i<_distrib_coeffs; i++){
    //    distrib.coeffs[i] += observation->distrib.coeffs[i];
    //}
}

//remove an observation from this object
void Observations::removeObservation(Observation* observation){     
    //riffle through the sightings until we find an identical pointer to this observation?
        //remove it, replace with the tail?
        //sightings.at(n) = pop_back(); //fast, breaks order
        //sightings.erase(n);   //slow, preserves order

}

//generate an distrib struct from a primitive and operators
Distrib generateDistrib(DistribParams params){
    //primitive is a, spherical distribution sigma=1
    Matx33d axes ( 
        0.5, 0, 0,
        0, 0.5, 0,
        0, 0, 0.5);
    Matx31d location (0,0,0);
    Distrib primitive = {axes,location};// = {0.5,0.5,0.5, 0,0,0, 0,0,0, 0};
//    for (int i = 0; i < 3; i++) {
//        primitive.coeffs[i] = 0.5;
//    }
//    for (int i = 3; i < _distrib_coeffs; i++) {
//        primitive.coeffs[i] = 0;
//    }
    Distrib dS = stretchDistrib(primitive, params.sigma_x, params.sigma_y, params.sigma_z);
    Distrib dR = rotateDistrib(dS, params.yaw, params.pitch, params.roll);
    Distrib dT = translateDistrib(dR, params.x, params.y, params.z);

    return dT;
}


//calculate the centre and covariance widths of this object
DistribParams getDistribParams(Distrib A){
    DistribParams D;
    //sigma values are the eigenvalues
    D.x = -A.location(0,0);
    D.y = -A.location(1,0);
    D.y = -A.location(2,0);
    return D;
}

//combine two distributions (as though statistically independent)
Distrib combineDistribs(Distrib A, Distrib B){
    Distrib C;
    //so easy in polynomial form, much harder in vector form
    //for(int i=0; i<_distrib_coeffs; i++){
    //    C.coeffs[i] = A.coeffs[i] + B.coeffs[i];
    //}
    return C;
}

//translate a distrib struct from the origin
Distrib translateDistrib(Distrib A, double x, double y, double z){
    cv::Matx31d offset(x,y,z);
    Distrib D;
    D.axes = A.axes;
    D.location = A.location - offset;
    return D;
}

//translate a distrib struct about the origin
Distrib rotateDistrib(Distrib A, double yaw, double pitch, double roll){
    Distrib D;
    double a;
    a = pitch;
    Matx33d Rx(
        1,      0,       0,
        0, cos(a), -sin(a),
        0, sin(a),  cos(a));
    a = roll;
    Matx33d Ry(
         cos(a), 0, sin(a),
              0, 1,      0,
        -sin(a), 0, cos(a));
    a = yaw;
    Matx33d Rz(
        cos(a), -sin(a), 0 , 
        sin(a),  cos(a), 0 , 
             0,       0, 1 ); 


    D.axes = Rx.inv() * Ry.inv() * Rz.inv() * A.axes * Rz * Ry * Rx;    //I think
    D.location = A.location;
    return D;
}
//stretch a distrib struct about the origin, only along the 
Distrib stretchDistrib(Distrib A, double sx, double sy, double sz){
    Distrib D;

    Matx33d S(
        sx, 0, 0,
        0, sy, 0,
        0, 0, sz);

    D.location = S * A.location;    //increase distance from origin
    D.axes = S.inv() * S.inv() * A.axes;    //increase the size

    return D;
}
