/**
 * @file observations.cpp
 * @brief Utility functions for manipulating and tracking uncertainties in observations
 */




#include "common.h"
#include "observations.h"

using namespace picopter;
using namespace cv;
using std::chrono::steady_clock;
using std::chrono::milliseconds;
using std::chrono::microseconds;
using std::chrono::seconds;
using std::chrono::duration_cast;
using std::this_thread::sleep_for;

namespace picopter{

Observations::Observations(Observation firstSighting) {

    accumulator = firstSighting;
    sightings.push_back(firstSighting);

//    appendObservation(firstSighting);
}


//estimate the probability the given observation is of the same object
//return the integral of  e^((x-l1).t()*A*(x-l1)) * e^((x-l2).t()*B*(x-l2))
double Observations::getSameProbability(Observation observation){ 
    //In many cases, we can't invert the matrices, directly sample them instead.
    Distrib C;
    C.axes = (accumulator.location.axes.inv() + observation.location.axes.inv()).inv();
    C.vect = accumulator.location.vect - observation.location.vect;
    double retval = exp(-(((C.vect).t() * (C.axes * C.vect))(0)));
    //double retval;
    //retval  = sampleDistrib(observation.location, accumulator.location.vect);
    //retval *= sampleDistrib(accumulator.location, observation.location.vect);
    
    return retval;
}

//add another sighting to this object
void Observations::appendObservation(Observation observation){
    sightings.push_back(observation);

    accumulator.location = combineDistribs(accumulator.location, observation.location);
    accumulator.velocity = combineDistribs(accumulator.velocity, observation.velocity);
    //update the time stamp
    if(observation.sample_time > accumulator.sample_time){
        accumulator.sample_time = observation.sample_time;
    }
    accumulator.source = observation.source;
}
void Observations::updateObject(TIME_TYPE timestep){

    //accumulator.location = vectorSum(accumulator.location, stretchDistrib(accumulator.velocity, ((double)(timestep.count()))/TICKS_PER_SEC ));
    accumulator.location = vectorSum(accumulator.location, stretchDistrib(accumulator.velocity, (duration_cast<microseconds>(timestep).count())/1000000.0 ));
    
    
    //velocity = vectorSum(velocity, stretchDistrib(acceleration, timestep));
    //acceleration = vectorSum(acceleration, something);

    
    //accumulator.sample_time += timestep;
    accumulator.source = INTERPOLATION;
}


//remove an observation from this object
void Observations::removeObservation(Observation observation){     
    //riffle through the sightings until we find an identical pointer to this observation?
        //remove it, replace with the tail?
        //sightings.at(n) = pop_back(); //fast, breaks order
        //sightings.erase(n);   //slow, preserves order

}

TIME_TYPE Observations::lastObservation(){
    return accumulator.sample_time;
}

Distrib Observations::getLocation(){
    return accumulator.location;
}


//generate an distrib struct from a primitive and operators

Distrib generatedistrib(){
    Matx33d axes ( 
        0.5, 0, 0,
        0, 0.5, 0,
        0, 0, 0.5);
    Vec3d vect (0,0,0);
    Distrib primitive = {axes,vect};
    return primitive;
}
Distrib generateDistrib(DistribParams params){
    //primitive is a, spherical distribution sigma=1
    // = e^-(0.5x^2 + 0.5y^2 + 0.5 z^2 + 0yz + 0xz + 0yx + 0x + 0y + 0z + 0);

    Distrib primitive = generatedistrib();
//    for (int i = 0; i < 3; i++) {
//        primitive.coeffs[i] = 0.5;
//    }
//    for (int i = 3; i < _distrib_coeffs; i++) {
//        primitive.coeffs[i] = 0;
//    }
    Distrib dS = stretchDistrib(primitive, params.sigma_x, params.sigma_y, params.sigma_z);
    Distrib dR = rotateDistribEuler(dS, params.yaw, params.pitch, params.roll);
    Distrib dT = translateDistrib(dR, params.x, params.y, params.z);

    return dT;
}


//calculate the centre and covariance widths of this object
DistribParams getDistribParams(Distrib A){
    DistribParams D;
    //sigma values are the eigenvalues
    D.x = -A.vect(0);
    D.y = -A.vect(1);
    D.y = -A.vect(2);
    return D;
}
double sampleDistrib(Distrib &A, Vec3d &B){
    return exp(-(((A.vect-B).t() * (A.axes * (A.vect-B)))(0)));
}

//combine two distributions (as though statistically independent, so beware of biases)
Distrib combineDistribs(Distrib A, Distrib B){
    Distrib C;
    C.axes = A.axes + B.axes;
    if(determinant(C.axes) <= DBL_MIN){
        std::cout << "Combine result is a non-invertible matrix!" << std::endl;
    }

    C.vect = C.axes.inv() * (A.axes * A.vect + B.axes * B.vect);    //takes two lines to prove
    //you can see here how if B is a zero matrix, it fully cancels out of this equation.
    //If no velocity or acceleration data is returned by a given sensor, the variance matrix is set to zeroes
    return C;
}

//translate a distrib struct from the origin

Distrib translateDistrib(Distrib A, Vec3d offset){
    Distrib D;
    D.axes = A.axes;
    D.vect = A.vect + offset;
    return D;
}


//rotate a distrib struct about the origin

Distrib rotateDistribEuler(Distrib A, double roll, double pitch, double yaw){
    Distrib D;
    Matx33d R_total = rotationMatrix(roll, pitch, yaw);
    return rotateDistrib(A, R_total);
}

Matx33d rotationMatrix(double roll, double pitch, double yaw){
    double a;
    a = DEG2RAD(roll);
    Matx33d Rx(1,      0,       0,
                    0, cos(a), -sin(a),
                    0, sin(a),  cos(a));
    a = DEG2RAD(pitch);
    Matx33d Ry( cos(a), 0,  sin(a),
                          0, 1,       0,
                    -sin(a), 0,  cos(a));
    a = DEG2RAD(yaw);
    Matx33d Rz(cos(a), -sin(a), 0,
                    sin(a),  cos(a), 0,
                        0,        0, 1);
    return Rz*Ry*Rx;
}

Distrib rotateDistrib(Distrib A, Matx33d Mrot){
    Distrib D;
    D.axes = Mrot * A.axes * Mrot.inv();
    D.vect = Mrot * A.vect;

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

    if((determinant(A.axes) <= DBL_MIN) || (determinant(B.axes) <= DBL_MIN)){
        std::cout << "vector sum non-invertible matrix!" << std::endl;
        return A;
    }
    
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

    estVel = stretchDistrib(estVel, TICKS_PER_SEC/(double)(timestep.count()));
    return estVel;
}


void rasterDistrib(Mat *mat, Distrib *dist, Vec4b colour, double scale)
{
    for (int i = 0; i < mat->rows; ++i) {
        for (int j = 0; j < mat->cols; ++j) {
            Vec4b& rgba = mat->at<Vec4b>(i, j);
            Vec4b C = colour;
            
            Vec3d B((j-(mat->cols/2.0))*scale, (i-(mat->rows/2.0))*scale,dist->vect[2]);    //always sample in the plane of the measurement

            C(0) = colour(0) * sampleDistrib(*dist, B);
            C(1) = colour(1) * sampleDistrib(*dist, B);
            C(2) = colour(2) * sampleDistrib(*dist, B);

            ////screen layering
            rgba[0] = saturate_cast<uchar>(
                (1.0 -(
                    (1.0-((double)C[0]/(double)UCHAR_MAX))*
                    (1.0-((double)rgba[0]/(double)UCHAR_MAX))
                 ))* UCHAR_MAX);
            rgba[1] = saturate_cast<uchar>(
                (1.0 -(
                    (1.0-((double)C[1]/(double)UCHAR_MAX))*
                    (1.0-((double)rgba[1]/(double)UCHAR_MAX))
                 ))* UCHAR_MAX);
            rgba[2] = saturate_cast<uchar>(
                (1.0 -(
                    (1.0-((double)C[2]/(double)UCHAR_MAX))*
                    (1.0-((double)rgba[2]/(double)UCHAR_MAX))
                 ))* UCHAR_MAX);
            rgba[3] = UCHAR_MAX;

            //rgba[0] = UCHAR_MAX;
            //rgba[1] = saturate_cast<uchar>((float (mat.cols - j)) / ((float)mat.cols) * UCHAR_MAX);
            //rgba[2] = saturate_cast<uchar>((float (mat.rows - i)) / ((float)mat.rows) * UCHAR_MAX);
            //rgba[3] = saturate_cast<uchar>(0.5 * (rgba[1] + rgba[2]));
        }
    }
}

void storeDistrib(Mat* mat, std::string filename){
    //Mat mat(rows, cols, CV_8UC4);
    //rasterDistrib(mat,dist);

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    imwrite(filename, *mat, compression_params);

    //std::cout <<  "Saved PNG file with alpha data." << std::endl;
}



}
