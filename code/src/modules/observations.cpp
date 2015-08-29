/**
 * @file observations.cpp
 * @brief Utility functions for manipulating and tracking uncertainties in observations
 */




#include "common.h"
#include "observations.h"

using namespace picopter;

Observations::Observations() {

}


//estimate the probability the given observation  is of the same object
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
    for(int i=0; i<_distrib_coeffs; i++){
        distrib.coeffs[i] += observation->distrib.coeffs[i];
    }

}

//remove an observation from this object
void Observations::removeObservation(Observation* observation){     


}

//generate an distrib struct from a primitive and operators
Distrib generateDistrib(DistribParams params){
    //primitive is a, spherical distribution sigma=1
    Distrib primitive;
    for (int i = 0; i < 3; i++) {
        primitive.coeffs[i] = 0.5;
    }
    for (int i = 3; i < _distrib_coeffs; i++) {
        primitive.coeffs[i] = 0;
    }
    
    Distrib dS = stretchDistrib(primitive, params.sigma_x, params.sigma_y, params.sigma_z);
    Distrib dR = rotateDistrib(dS, params.yaw, params.pitch, params.roll);
    Distrib dT = translateDistrib(dR, params.x, params.y, params.z);

    return dT;
}


DistribParams getDistribParams(Distrib A);                  //calculate the centre and covariance widths of this object

//combine two distributions (as though statistically independent)
Distrib combineDistribs(Distrib A, Distrib B){
    Distrib C;
    for(int i=0; i<_distrib_coeffs; i++){
        C.coeffs[i] = A.coeffs[i] + B.coeffs[i];
    }
    return C;
}

//translate a distrib struct from the origin
Distrib translateDistrib(Distrib A, double x, double y, double z){
    Distrib D = {0};
    return D;
}

//translate a distrib struct about the origin
Distrib rotateDistrib(Distrib A, double yaw, double pitch, double roll){
    Distrib D = {0};
    return D;
}
//stretch a distrib struct about the origin
Distrib stretchDistrib(Distrib A, double sx, double sy, double sz){
    Distrib D = {0};
    return D;
}
