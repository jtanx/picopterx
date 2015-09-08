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
//return the integral of  e^((x-l1).t()*A*(x-l1)) * e^((x-l2).t()*B*(x-l2))
double Observations::getSameProbability(Observation* observation){ 
    Distrib C;
    C.axes = (location.axes.inv() + observation->location.axes.inv()).inv();
    C.vect = location.vect - observation->location.vect;
    double retval = ((C.vect).t() * (C.axes * C.vect))(0,0);


return retval;
}


//add another sighting to this object
void Observations::appendObservation(Observation* observation){     
    sightings.push_back(observation);
    //for(int i=0; i<_distrib_coeffs; i++){
    //    distrib.coeffs[i] += observation->distrib.coeffs[i];
    //}
    location = combineDistribs(location, observation->location);
    velocity = combineDistribs(velocity, observation->velocity);
}
void Observations::updateObject(TIME_TYPE timestep){
    location = vectorSum(location, stretchDistrib(velocity, timestep));
    velocity = vectorSum(velocity, stretchDistrib(acceleration, timestep));
    //acceleration = vectorSum(acceleration, something);
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
    // = e^-(0.5x^2 + 0.5y^2 + 0.5 z^2 + 0yz + 0xz + 0yx + 0x + 0y + 0z + 0);
    Matx33d axes ( 
        0.5, 0, 0,
        0, 0.5, 0,
        0, 0, 0.5);
    Matx31d vect (0,0,0);
    Distrib primitive = {axes,vect};
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
    D.x = -A.vect(0,0);
    D.y = -A.vect(1,0);
    D.y = -A.vect(2,0);
    return D;
}

//combine two distributions (as though statistically independent, so beware of biases)
Distrib combineDistribs(Distrib A, Distrib B){
    Distrib C;
    //so easy in polynomial form.
    //for(int i=0; i<_distrib_coeffs; i++){
    //    C.coeffs[i] = A.coeffs[i] + B.coeffs[i];
    //}
    C.axes = A.axes + B.axes;
    C.vect = C.axes.inv() * (A.axes * A.vect + B.axes * B.vect);    //takes two lines to prove
    //you can see here how if B is a zero matrix, it fully cancels out of this equation.
    //If no velocity or acceleration data is returned by a given sensor, the variance matrix is set to zeroes
    return A;
}

//translate a distrib struct from the origin
Distrib translateDistrib(Distrib A, double x, double y, double z){
    cv::Matx31d offset(x,y,z);
    Distrib D;
    D.axes = A.axes;
    D.vect = A.vect - offset;
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

//    D.axes = Rx.inv() * Ry.inv() * Rz.inv() * A.axes * Rz * Ry * Rx;
    D.axes = Rx.t() * Ry.t() * Rz.t() * A.axes * Rz * Ry * Rx;    //makes more sense as transposes
    D.vect = Rz * Ry * Rx * A.vect;
    return D;
}

//stretch a distrib struct about the origin by a ratio
Distrib stretchDistrib(Distrib A, double sx, double sy, double sz){
    Distrib D;

    Matx33d S(
        sx, 0, 0,
        0, sy, 0,
        0, 0, sz);

    D.vect = S * A.vect;    //increase distance from origin
    D.axes = S.inv() * S.inv() * A.axes;    //increase the size

    return D;
}
    
    //we want the convolution of e^((x-l1).t()*A*(x-l1)), e^((x-l2).t()*B*(x-l2))
    //The translation of one Distrib by another. Can be used to encode velocity uncertainty
Distrib vectorSum(Distrib A, Distrib B){
    Distrib C;
    C.vect = A.vect + B.vect;
    C.axes = (A.axes.inv() + B.axes.inv()).inv();
    return C;
}

    //estimate the velocity from two locations and a time difference
    //estimate the acceleration from two velocities and a time difference
    //This is basicaly the same as the vectorSum. These won't make reversible transformations
Distrib changeStep(Distrib newLoc, Distrib oldLoc, TIME_TYPE timestep){
    Distrib estVel;
    estVel.vect = newLoc.vect - oldLoc.vect;    //difference in position
    estVel.axes = (newLoc.axes.inv() + oldLoc.axes.inv()).inv();    //change in variance is same as vectorSum

    estVel = stretchDistrib(estVel, 1/timestep);
    return estVel;
}